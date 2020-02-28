use crate::cdc_acm::*;
use crate::serial_port::{WriteState, SHORT_PACKET_INTERVAL};
use core::{marker::PhantomData, mem::MaybeUninit, ptr, slice};
use generic_array::{
    typenum::{consts::*, marker_traits::Unsigned},
    ArrayLength, GenericArray,
};
use heapless::pool::singleton::{Box, Pool};
use usb_device::class_prelude::*;
use usb_device::UsbError;

#[derive(Copy, Clone, Debug)]
pub enum PoolError {
    OutOfMemory,
    BufferFull,
    UsbInternalError,
}

pub struct PoolNode<N = U64>
where
    N: ArrayLength<MaybeUninit<u8>> + Unsigned + 'static,
{
    len: usize,
    buf: GenericArray<MaybeUninit<u8>, N>,
}

impl PoolNode {
    /// Creates a new node with 64 bytes of total space
    pub fn new_default() -> Self {
        Self {
            len: 0,
            buf: unsafe {
                #[allow(clippy::uninit_assumed_init)]
                MaybeUninit::uninit().assume_init()
            },
        }
    }
}

// Heavily inspired by @korken89 work
impl<N> PoolNode<N>
where
    N: ArrayLength<MaybeUninit<u8>> + Unsigned + 'static,
{
    /// Creates a new node
    pub fn new() -> Self {
        Self {
            len: 0,
            buf: unsafe {
                #[allow(clippy::uninit_assumed_init)]
                MaybeUninit::uninit().assume_init()
            },
        }
    }

    /// Gives a `&mut [u8]` slice to write into with the maximum size, the `commit` method
    /// must then be used to set the actual number of bytes written.
    ///
    /// Note that this function internally first zeros the node's buffer.
    pub fn write(&mut self) -> &mut [u8] {
        // Initialize memory with a safe value
        for elem in self.buf.iter_mut() {
            *elem = MaybeUninit::zeroed();
        }
        self.len = N::USIZE; // Set to max so `commit` may shrink it if needed

        unsafe {
            slice::from_raw_parts_mut(self.buf.as_mut_slice().as_mut_ptr() as *mut _, N::USIZE)
        }
    }

    /// Used to shrink the current size of the slice in the node, mostly used in conjunction
    /// with `write`.
    pub fn commit(&mut self, shrink_to: usize) {
        // Only shrinking is allowed to remain safe with the `MaybeUninit`
        if shrink_to < self.len {
            self.len = shrink_to;
        }
    }

    /// Used to write data into the node, and returns how many bytes were written from `buf`.
    ///
    /// If the node is already partially filled, this will continue filling the node.
    pub fn write_slice(&mut self, buf: &[u8]) -> usize {
        let free = N::USIZE - self.len as usize;
        let new_size = buf.len();
        let count = if new_size > free { free } else { new_size };

        // Used to write data into the `MaybeUninit`, safe based on the size check above
        unsafe {
            ptr::copy_nonoverlapping(
                buf.as_ptr(),
                self.buf.as_mut_slice().as_mut_ptr().add(self.len) as *mut u8,
                count,
            );
        }

        self.len += count;
        count
    }

    /// Clear the node of all data making it empty
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Returns a readable slice which maps to the buffers internal data
    pub fn read(&self) -> &[u8] {
        // Safe as it uses the internal length of valid data
        unsafe {
            slice::from_raw_parts(self.buf.as_slice().as_ptr() as *const _, self.len as usize)
        }
    }

    /// Reads how many bytes are available
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// Checks if the node is empty
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    //unsafe fn set_len(&mut self, len: usize) {
    //    self.len = len;
    //}
    //unsafe fn buffer_address_for_dma(&self) -> u32 {
    //    self.buf.as_slice().as_ptr() as u32
    //}

    pub fn max_len() -> usize {
        N::USIZE
    }
}

pub struct PoolPort<'a, B, PT, PR, NT, NR>
where
    NT: ArrayLength<MaybeUninit<u8>> + Unsigned + 'static,
    NR: ArrayLength<MaybeUninit<u8>> + Unsigned + 'static,
    B: UsbBus,
    PT: Pool<Data = PoolNode<NT>>,
    PR: Pool<Data = PoolNode<NR>>,
{
    inner: CdcAcmClass<'a, B>,
    write_state: WriteState,
    _pool_t: PhantomData<PT>,
    _pool_x: PhantomData<PR>,
    write_buf: Option<Box<PT>>,
    written_count: usize,
    read_buf: Option<Box<PR>>,
    read_count: usize,
}

impl<B, PT, PR, NT, NR> PoolPort<'_, B, PT, PR, NT, NR>
where
    NT: ArrayLength<MaybeUninit<u8>> + Unsigned + 'static,
    NR: ArrayLength<MaybeUninit<u8>> + Unsigned + 'static,
    B: UsbBus,
    PT: Pool<Data = PoolNode<NT>>,
    PR: Pool<Data = PoolNode<NR>>,
{
    /// Creates a new usb serial port, if the buffers are not provided it will try to allocate them,
    /// returning `Err(PoolError::OutOfMemory)` if it fails
    pub fn new(
        alloc: &UsbBusAllocator<B>,
        write_buf: Option<Box<PT>>,
        read_buf: Option<Box<PR>>,
    ) -> Result<PoolPort<'_, B, PT, PR, NT, NR>, PoolError> {
        let wb = if write_buf.is_some() {
            write_buf
        } else {
            Some(
                PT::alloc()
                    .ok_or(PoolError::OutOfMemory)?
                    .init(PoolNode::new()),
            )
        };
        let rb = if read_buf.is_some() {
            read_buf
        } else {
            Some(
                PR::alloc()
                    .ok_or(PoolError::OutOfMemory)?
                    .init(PoolNode::new()),
            )
        };
        Ok(PoolPort {
            inner: CdcAcmClass::new(alloc, 64),
            write_state: WriteState::Idle,
            _pool_t: PhantomData,
            _pool_x: PhantomData,
            write_buf: wb,
            written_count: 0,
            read_buf: rb,
            read_count: 0,
        })
    }

    /// Gets the current line coding.
    pub fn line_coding(&self) -> &LineCoding {
        self.inner.line_coding()
    }

    /// Gets the DTR (data terminal ready) state
    pub fn dtr(&self) -> bool {
        self.inner.dtr()
    }

    /// Gets the RTS (ready to send) state
    pub fn rts(&self) -> bool {
        self.inner.rts()
    }

    /// Checks if the writer buffer is empty
    pub fn writer_empty(&self) -> bool {
        self.write_buf.is_none()
    }

    /// Returns the length of the writer buffer
    pub fn writer_len(&self) -> usize {
        if let Some(ref buf) = self.write_buf {
            buf.len()
        } else {
            0
        }
    }

    /// Replaces the writer buffer returning the old value if present
    pub fn replace_writer(&mut self, new: Option<Box<PT>>) -> Option<Box<PT>> {
        self.written_count = 0;
        let old = self.write_buf.take();
        self.write_buf = new;
        old
    }

    /// Checks if the reader buffer is empty
    pub fn reader_empty(&self) -> bool {
        self.read_buf.is_none()
    }

    /// Returns the length of the reader buffer
    pub fn reader_len(&self) -> usize {
        if let Some(ref buf) = self.read_buf {
            buf.len()
        } else {
            0
        }
    }

    /// Replaces the reader buffer returning the old value if present
    pub fn replace_reader(&mut self, new: Option<Box<PR>>) -> Option<Box<PR>> {
        self.read_count = 0;
        let old = self.read_buf.take();
        self.read_buf = new;
        old
    }

    /// Writes bytes from `data` into the port and returns the number of bytes written.
    pub fn write(&mut self, data: &[u8]) -> Result<usize, PoolError> {
        if data.is_empty() {
            return Ok(0);
        }
        if self.write_buf.is_none() {
            self.write_buf = Some(
                PT::alloc()
                    .ok_or(PoolError::OutOfMemory)?
                    .init(PoolNode::new()),
            );
            self.written_count = 0;
        }
        let buf = self.write_buf.as_mut().unwrap();
        let count = buf.write_slice(data);

        match self.flush() {
            Ok(_) | Err(UsbError::WouldBlock) => {}
            Err(_) => {
                return Err(PoolError::UsbInternalError);
            }
        };

        if count == 0 {
            Err(PoolError::BufferFull)
        } else {
            Ok(count)
        }
    }

    /// Copies the data from usb internal buffers to the Port buffer, returns true if any data was
    /// available
    pub fn process(&mut self) -> Result<usize, PoolError> {
        if self.read_buf.is_none() {
            self.read_buf = Some(
                PR::alloc()
                    .ok_or(PoolError::OutOfMemory)?
                    .init(PoolNode::new()),
            );
            self.read_count = 0;
        }
        let buf = self.read_buf.as_mut().unwrap();
        if self.read_count < PoolNode::<NR>::max_len() {
            let count = match self.inner.read_packet(&mut buf.write()[self.read_count..]) {
                Ok(c) => c,
                Err(UsbError::WouldBlock) => 0,
                Err(_) => return Err(PoolError::UsbInternalError),
            };
            self.read_count += count;
            buf.commit(self.read_count);
            Ok(count)
        } else {
            Err(PoolError::BufferFull)
        }
    }

    /// Returns a reference to the writer buffer if it exists
    pub fn writer_buf(&self) -> Option<&[u8]> {
        if let Some(buf) = &self.write_buf {
            Some(buf.read())
        } else {
            None
        }
    }

    /// Returns a reference to the reader buffer if it exists
    pub fn reader_buf(&self) -> Option<&[u8]> {
        if let Some(buf) = &self.read_buf {
            Some(buf.read())
        } else {
            None
        }
    }

    /// Sends as much as possible of the current write buffer. Returns `Ok` if all data that has
    /// been written has been completely written to hardware buffers `Err(WouldBlock)` if there is
    /// still data remaining, and other errors if there's an error sending data to the host. Note
    /// that even if this method returns `Ok`, data may still be in hardware buffers on either side.
    pub fn flush(&mut self) -> Result<(), UsbError> {
        let inner = &mut self.inner;
        let write_state = &mut self.write_state;
        let full_count = match *write_state {
            WriteState::Full(c) => c,
            _ => 0,
        };

        if self.write_buf.is_none() {
            if full_count != 0 {
                // Write a ZLP to complete the transaction if there's nothing else to write and the last
                // packet was a full one. This may return WouldBlock which will be propagated.
                inner.write_packet(&[])?;

                *write_state = WriteState::Short;

                return Err(UsbError::WouldBlock);
            } else {
                // No data left in writer_buf.
                *write_state = WriteState::Idle;
                return Ok(());
            }
        }
        // if we are here there is a available write_buf
        let buf = self.write_buf.as_ref().unwrap();
        if buf.len() == 0 {
            // There is nothing in the buffer, drop it to free memory on the pool
            self.write_buf = None;
            self.written_count = 0;
            return Ok(());
        // We don't really need this check if `written_count` isn't increased somewhere else
        } else if buf.len() > self.written_count {
            let max_write_size = if full_count >= SHORT_PACKET_INTERVAL {
                inner.max_packet_size() - 1
            } else {
                inner.max_packet_size()
            } as usize;

            let write_size = core::cmp::min(max_write_size, buf.len() - self.written_count);
            // This may return WouldBlock which will be propagated.
            let written = inner
                .write_packet(&buf.read()[self.written_count..self.written_count + write_size])?;
            self.written_count += written;
            if self.written_count >= buf.len() {
                self.write_buf = None;
                self.written_count = 0;
            }
            *write_state = if written == inner.max_packet_size() as usize {
                WriteState::Full(full_count + 1)
            } else {
                WriteState::Short
            };
            return Err(UsbError::WouldBlock);
        }
        Err(UsbError::WouldBlock)
    }
}

impl<B, PT, PR, NT, NR> UsbClass<B> for PoolPort<'_, B, PT, PR, NT, NR>
where
    NT: ArrayLength<MaybeUninit<u8>> + Unsigned + 'static,
    NR: ArrayLength<MaybeUninit<u8>> + Unsigned + 'static,
    B: UsbBus,
    PT: Pool<Data = PoolNode<NT>>,
    PR: Pool<Data = PoolNode<NR>>,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<(), UsbError> {
        self.inner.get_configuration_descriptors(writer)
    }

    fn reset(&mut self) {
        self.inner.reset();
        self.replace_reader(None);
        self.replace_writer(None);
        self.write_state = WriteState::Idle;
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.inner.write_ep_address() {
            self.flush().ok();
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        self.inner.control_in(xfer);
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        self.inner.control_out(xfer);
    }
}

#[cfg(test)]
mod tests {
    //extern crate std;

    use crate::pool_serial::PoolNode;

    const DATA: &[u8] = &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
    #[test]
    fn write_read() {
        let mut node = PoolNode::new_default();
        let written = node.write_slice(DATA);
        assert_eq!(written, 12);
        assert_eq!(node.len(), 12);
        assert_eq!(node.read(), DATA);
    }
}
