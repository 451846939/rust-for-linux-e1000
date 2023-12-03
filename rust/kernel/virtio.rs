// SPDX-License-Identifier: GPL-2.0

//! PCI devices and drivers.
//!
//! C header: [`include/linux/virtio.h`](../../../../include/linux/virtio.h)

use crate::device::Device;
use crate::sync::Arc;
use crate::{
    bindings, device, driver,
    error::{
        code::{EINVAL, ENOMEM},
        from_kernel_result, Error, Result,
    },
    str::CStr,
    to_result,
    types::PointerWrapper,
    ThisModule,
};

#[derive(Clone, Copy)]
pub struct DeviceId {
    pub device: u32,
    pub vendor: u32,
}

impl DeviceId {
    pub const fn new(device:u32,vendor:u32) ->Self{
        Self { device, vendor }
    }
}

pub struct VirtioDevice {
    pub(crate) ptr: *mut bindings::virtio_device,
}

impl VirtioDevice {
    unsafe fn from_ptr(ptr: *mut bindings::virtio_device) -> Self {
        Self { ptr }
    }

    pub fn config_get(
        &self
    ) -> Option<unsafe extern "C" fn(*mut bindings::virtio_device, u32, *mut core::ffi::c_void, u32)> {
        unsafe { (*(*self.ptr).config).get }
    }
}

// SAFETY: `ZERO` is all zeroed-out and `to_rawid` stores `offset` in `pci_device_id::driver_data`.
unsafe impl const driver::RawDeviceId for DeviceId {
    type RawType = bindings::virtio_device_id;

    const ZERO: Self::RawType = bindings::virtio_device_id {
        vendor: 0,
        device: 0,
    };

    fn to_rawid(&self, offset: isize) -> Self::RawType {
        bindings::virtio_device_id {
            vendor: self.vendor,
            device: self.device,
        }
    }
}
pub trait Driver {
    /// The type holding information about each device id supported by the driver.
    type IdInfo: 'static = ();
    /// The table of device ids supported by the driver.
    const ID_TABLE: driver::IdTable<'static, DeviceId, Self::IdInfo>;

    fn probe(dev: Arc<VirtioDevice>) -> Result<()>;

    fn remove(_data: &mut VirtioDevice);
}

pub struct Adapter<T: Driver>(T);

impl<T: Driver> Adapter<T> {
    extern "C" fn probe_callback(vdev: *mut bindings::virtio_device) -> core::ffi::c_int {
        // from_kernel_result! {
        //     // SAFETY: `pdev` is valid by the contract with the C code. `dev` is alive only for the
        //     // duration of this call, so it is guaranteed to remain alive for the lifetime of
        //     // `pdev`.
        let mut dev = unsafe { VirtioDevice::from_ptr(vdev) };
        //     // SAFETY: `id` is a pointer within the static table, so it's always valid.
        //     let offset = unsafe {(*id).driver_data};
        //     // SAFETY: The offset comes from a previous call to `offset_from` in `IdArray::new`, which
        //     // guarantees that the resulting pointer is within the table.
        //     let info = {
        //         let ptr = unsafe {id.cast::<u8>().offset(offset as _).cast::<Option<T::IdInfo>>()};
        //         unsafe {(&*ptr).as_ref()}
        //     };
        //     let data = T::probe(&mut dev, info)?;
        //      // SAFETY: `pdev` is guaranteed to be a valid, non-null pointer.
        //     unsafe { bindings::pci_set_drvdata(pdev, data.into_pointer() as _) };
        //     Ok(0)
        // }
        T::probe(Arc::try_new(dev).unwrap());
        // T.probe(&mut T, &mut dev);
        0
    }

    extern "C" fn remove_callback(pdev: *mut bindings::virtio_device) {
        let mut dev = unsafe { VirtioDevice::from_ptr(pdev) };

        T::remove(&mut dev);
    }

    // extern "C" fn restore_callback(pdev: *mut bindings::pci_dev)-> core::ffi::c_int{
    //
    // }
}

impl<T: Driver> driver::DriverOps for Adapter<T> {
    type RegType = bindings::virtio_driver;

    unsafe fn register(
        reg: *mut Self::RegType,
        name: &'static CStr,
        module: &'static ThisModule,
    ) -> Result {
        let virtio_driver: &mut bindings::virtio_driver = unsafe { &mut *reg };

        // 这里如果要去掉virtio.c的作用的话还需要封装bus的注册，这里先暂时直接使用
        // virtio_driver.driver.bus=

        virtio_driver.probe = Some(Self::probe_callback);
        virtio_driver.remove = Some(Self::remove_callback);
        // virtio_driver.restore=Some(Self::restore_callback);
        // virtio_driver.freeze=Some(Self::freeze_callback);
        // virtio_driver.validate=Some(Self::validate_callback);
        // virtio_driver.scan=Some(Self::scan_callback);

        virtio_driver.id_table = T::ID_TABLE.as_ref();
        unsafe {
            bindings::register_virtio_driver(virtio_driver);
        }
        Ok(())
    }

    unsafe fn unregister(reg: *mut Self::RegType) {
        // SAFETY: `reg` is a valid pointer to a `virtio_driver` struct.
        unsafe { bindings::unregister_virtio_driver(reg) }
    }
}

#[macro_export]
macro_rules! define_virtio_id_table {
    ($data_type:ty, $($t:tt)*) => {
        type IdInfo = $data_type;
        const ID_TABLE: $crate::driver::IdTable<'static, $crate::virtio::DeviceId, $data_type> = {
            $crate::define_id_array!(ARRAY, $crate::virtio::DeviceId, $data_type, $($t)* );
            ARRAY.as_table()
        };
    };
}
