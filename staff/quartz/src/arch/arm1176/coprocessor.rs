#[macro_export]
macro_rules! define_coprocessor_register {
    ($name:ident, $p:ident, $op1:literal, $crn:ident, $crm:ident, $op2:literal) => {
        pub mod $name {
            $crate::define_coprocessor_register!(@const $name, $p, $op1, $crn, $crm, $op2);
            $crate::define_coprocessor_register!(@mut $name, $p, $op1, $crn, $crm, $op2);
        }
    };
    (read $name:ident, $p:ident, $op1:literal, $crn:ident, $crm:ident, $op2:literal) => {
        pub mod $name {
            $crate::define_coprocessor_register!(@const $name, $p, $op1, $crn, $crm, $op2);
        }
    };
    (write $name:ident, $p:ident, $op1:literal, $crn:ident, $crm:ident, $op2:literal) => {
        pub mod $name {
            $crate::define_coprocessor_register!(@mut $name, $p, $op1, $crn, $crm, $op2);
        }
    };
    (@const $name:ident, $p:ident, $op1:literal, $crn:ident, $crm:ident, $op2:literal) => {
        #[allow(unused)]
        pub unsafe fn read() -> usize {
            let mut out : usize;
            unsafe { ::core::arch::asm!(
                     concat!("mrc ",stringify!($p),", ",$op1,", {tmp}, ",stringify!($crn),", ",stringify!($crm),", ",$op2),
                     tmp = out(reg) out) };
            out
        }
    };
    (@mut $name:ident, $p:ident, $op1:literal, $crn:ident, $crm:ident, $op2:literal) => {
        #[allow(unused)]
        pub unsafe fn write(arg: usize) {
            unsafe { ::core::arch::asm!(
                     concat!("mcr ",stringify!($p),", ",$op1,", {tmp}, ",stringify!($crn),", ",stringify!($crm),", ",$op2),
                     tmp = in(reg) arg) };
        }
    };
}

#[macro_export]
macro_rules! define_coprocessor_registers {
    {
        $(
            $([ $($modifiers:ident)* $(,)? ])?
            $name:ident
            $(: $($as_ty:ident)::+)?
            => $cp:ident $op0:literal $crn:ident $crm:ident $op1:literal
        );*
        $(;)?
    } => {
        $(
        pub mod $name {
            $crate::define_coprocessor_registers!(@type $($($as_ty)::+)?);
            $crate::define_coprocessor_registers!(@unsafe read @mod($($($modifiers),*)?) $(as $($as_ty)::+)? => $cp $op0 $crn $crm $op1);
            $crate::define_coprocessor_registers!(@unsafe write @mod($($($modifiers),*)?) $(as $($as_ty)::+)? => $cp $op0 $crn $crm $op1);
            $crate::define_coprocessor_registers!(@unsafe both @mod($($($modifiers),*)?) $(as $($as_ty)::+)?);
        }
        )*
    };

    (@type $as_ty:ident) => {
        use super::$as_ty;
        pub type Ty = $as_ty;
    };
    (@type $($as_ty:ident)::+) => {
        pub type Ty = $($as_ty)::+;
    };
    (@type) => { pub type Ty = u32; };
    (@unsafe read @mod(safe$(, read)?) $(as $($as_ty:ident)::+)? => $cp:ident $op0:literal $crn:ident $crm:ident $op1:literal) => {
        pub fn read_raw() -> u32 {
            let mut out : u32;
            $crate::define_coprocessor_registers!(@for read(out) $cp $op0 $crn $crm $op1);
            out
        }
        $(
            pub fn read() -> $($as_ty)::+ { <$($as_ty)::+ as ::core::convert::From<u32>>::from(self::read_raw()) }
        )?
    };
    (@unsafe read @mod($(read)?) $(as $($as_ty:ident)::+)? => $cp:ident $op0:literal $crn:ident $crm:ident $op1:literal) => {
        pub unsafe fn read_raw() -> u32 {
            let mut out : u32;
            $crate::define_coprocessor_registers!(@for read(out) $cp $op0 $crn $crm $op1);
            out
        }
        $(
            pub unsafe fn read() -> $($as_ty)::+ { unsafe { <$($as_ty)::+ as ::core::convert::From<u32>>::from(self::read_raw()) } }
        )?
    };
    (@unsafe write @mod(safe$(, write)?) $(as $($as_ty:ident)::+)? => $cp:ident $op0:literal $crn:ident $crm:ident $op1:literal) => {
        pub fn write_raw(value: u32) {
            $crate::define_coprocessor_registers!(@for write(value) $cp $op0 $crn $crm $op1);
        }
        $(
            pub fn write(value: $($as_ty)::+) {
                self::write_raw(<$($as_ty)::+ as ::core::convert::Into<u32>>::into(value))
            }
        )?
    };
    (@unsafe write @mod($(write)?) $(as $($as_ty:ident)::+)? => $cp:ident $op0:literal $crn:ident $crm:ident $op1:literal) => {
        pub unsafe fn write_raw(value: u32) {
            $crate::define_coprocessor_registers!(@for write(value) $cp $op0 $crn $crm $op1);
        }
        $(
            pub unsafe fn write(value: $($as_ty)::+) {
                unsafe { self::write_raw(<$($as_ty)::+ as ::core::convert::Into<u32>>::into(value)) }
            }
        )?
    };
    (@unsafe read @mod($(safe ,)?write) $($rest:tt)*) => {};
    (@unsafe write @mod($(safe ,)?read) $($rest:tt)*) => {};
    (@unsafe both @mod(safe) $(as $($as_ty:ident)::+)?) => {
        pub fn modify_raw(f: impl FnOnce(u32) -> u32) {
            self::write_raw(f(self::read_raw()));
        }
        $(
        pub fn modify(f: impl FnOnce($($as_ty)::+) -> self::Ty) {
            self::write(f(self::read()));
        }
        )?
    };
    (@unsafe both @mod() $(as $($as_ty:ident)::+)?) => {
        pub unsafe fn modify_raw(f: impl FnOnce(u32) -> u32) {
            unsafe { self::write_raw(f(self::read_raw())) };
        }
        $(
        pub unsafe fn modify(f: impl FnOnce($($as_ty)::+) -> self::Ty) {
            unsafe { self::write(f(self::read())) };
        }
        )?
    };
    (@unsafe both @mod($(safe ,)?$(read)?$(write)?) $($rest:tt)*) => {};
    (@for read($output:ident) $cp:ident $op0:literal $crn:ident $crm:ident $op1:literal) => {
        unsafe { ::core::arch::asm!(
                 concat!("mrc ",stringify!($cp),", ",$op0,", {tmp}, ",stringify!($crn),", ",stringify!($crm),", ",$op1),
                 tmp = out(reg) $output) };
    };
    (@for write($input:ident) $cp:ident $op0:literal $crn:ident $crm:ident $op1:literal) => {
        unsafe { ::core::arch::asm!(
                 concat!("mcr ",stringify!($cp),", ",$op0,", {tmp}, ",stringify!($crn),", ",stringify!($crm),", ",$op1),
                 tmp = in(reg) $input) };
    };
}
