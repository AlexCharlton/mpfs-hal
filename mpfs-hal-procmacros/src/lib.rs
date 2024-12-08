use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

//-------------------------------------------------------------
// (Non-embassy) Entry point macros
/*
Macro usage:
#[hart1_main]
fn hart1_main() {
    <main_body>
}

Expands to:
#[no_mangle]
pub fn __hart1_entry() {
    <main_body>
}
*/

#[proc_macro_attribute]
pub fn hart1_main(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(item as ItemFn);
    let fn_body = &input_fn.block;

    let expanded = quote! {
        #[no_mangle]
        pub fn __hart1_entry() {
            #fn_body
        }
    };

    expanded.into()
}

#[proc_macro_attribute]
pub fn hart2_main(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(item as ItemFn);
    let fn_body = &input_fn.block;

    let expanded = quote! {
        #[no_mangle]
        pub fn __hart2_entry() {
            #fn_body
        }
    };

    expanded.into()
}

#[proc_macro_attribute]
pub fn hart3_main(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(item as ItemFn);
    let fn_body = &input_fn.block;

    let expanded = quote! {
        #[no_mangle]
        pub fn __hart3_entry() {
            #fn_body
        }
    };

    expanded.into()
}

#[proc_macro_attribute]
pub fn hart4_main(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(item as ItemFn);
    let fn_body = &input_fn.block;

    let expanded = quote! {
        #[no_mangle]
        pub fn __hart4_entry() {
            #fn_body
        }
    };

    expanded.into()
}

//-------------------------------------------------------------
// Embassy macros
/*
// Macro usage:
#[embassy_hart1_main]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    <task_body>
}


 // Expands to:
#[no_mangle]
fn __hart1_entry() {
    static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
    EXECUTOR1.init(Executor::new()).run(|spawner| {
        spawner.must_spawn(hart1_main(spawner));
    });
}

#[embassy_executor::task]
async fn hart1_main(_spawner: embassy_executor::Spawner) {
    <task_body>
}
 */

#[cfg(feature = "embassy")]
#[proc_macro_attribute]
pub fn embassy_hart1_main(_attr: TokenStream, item: TokenStream) -> TokenStream {
    // Parse the input function
    let input_fn = parse_macro_input!(item as ItemFn);

    // Extract the original function name and body
    let fn_name = &input_fn.sig.ident;
    let fn_body = &input_fn.block;
    let fn_inputs = &input_fn.sig.inputs;

    // Generate the expanded code
    let expanded = quote! {
        #[no_mangle]
        fn __hart1_entry() {
            static EXECUTOR1: ::mpfs_hal_embassy::static_cell::StaticCell<::mpfs_hal_embassy::Executor> =
                ::mpfs_hal_embassy::static_cell::StaticCell::new();

            EXECUTOR1
                .init(::mpfs_hal_embassy::Executor::new())
                .run(|spawner| {
                    spawner.must_spawn(#fn_name(spawner));
                });
        }

        #[::embassy_executor::task]
        async fn #fn_name(#fn_inputs) #fn_body
    };

    expanded.into()
}

#[cfg(feature = "embassy")]
#[proc_macro_attribute]
pub fn embassy_hart2_main(_attr: TokenStream, item: TokenStream) -> TokenStream {
    // Parse the input function
    let input_fn = parse_macro_input!(item as ItemFn);

    // Extract the original function name and body
    let fn_name = &input_fn.sig.ident;
    let fn_body = &input_fn.block;
    let fn_inputs = &input_fn.sig.inputs;

    // Generate the expanded code
    let expanded = quote! {
        #[no_mangle]
        fn __hart2_entry() {
            static EXECUTOR1: ::mpfs_hal_embassy::static_cell::StaticCell<::mpfs_hal_embassy::Executor> =
                ::mpfs_hal_embassy::static_cell::StaticCell::new();

            EXECUTOR1
                .init(::mpfs_hal_embassy::Executor::new())
                .run(|spawner| {
                    spawner.must_spawn(#fn_name(spawner));
                });
        }

        #[::embassy_executor::task]
        async fn #fn_name(#fn_inputs) #fn_body
    };

    expanded.into()
}

#[cfg(feature = "embassy")]
#[proc_macro_attribute]
pub fn embassy_hart3_main(_attr: TokenStream, item: TokenStream) -> TokenStream {
    // Parse the input function
    let input_fn = parse_macro_input!(item as ItemFn);

    // Extract the original function name and body
    let fn_name = &input_fn.sig.ident;
    let fn_body = &input_fn.block;
    let fn_inputs = &input_fn.sig.inputs;

    // Generate the expanded code
    let expanded = quote! {
        #[no_mangle]
        fn __hart3_entry() {
            static EXECUTOR1: ::mpfs_hal_embassy::static_cell::StaticCell<::mpfs_hal_embassy::Executor> =
                ::mpfs_hal_embassy::static_cell::StaticCell::new();

            EXECUTOR1
                .init(::mpfs_hal_embassy::Executor::new())
                .run(|spawner| {
                    spawner.must_spawn(#fn_name(spawner));
                });
        }

        #[::embassy_executor::task]
        async fn #fn_name(#fn_inputs) #fn_body
    };

    expanded.into()
}

#[cfg(feature = "embassy")]
#[proc_macro_attribute]
pub fn embassy_hart4_main(_attr: TokenStream, item: TokenStream) -> TokenStream {
    // Parse the input function
    let input_fn = parse_macro_input!(item as ItemFn);

    // Extract the original function name and body
    let fn_name = &input_fn.sig.ident;
    let fn_body = &input_fn.block;
    let fn_inputs = &input_fn.sig.inputs;

    // Generate the expanded code
    let expanded = quote! {
        #[no_mangle]
        fn __hart4_entry() {
            static EXECUTOR1: ::mpfs_hal_embassy::static_cell::StaticCell<::mpfs_hal_embassy::Executor> =
                ::mpfs_hal_embassy::static_cell::StaticCell::new();

            EXECUTOR1
                .init(::mpfs_hal_embassy::Executor::new())
                .run(|spawner| {
                    spawner.must_spawn(#fn_name(spawner));
                });
        }

        #[::embassy_executor::task]
        async fn #fn_name(#fn_inputs) #fn_body
    };

    expanded.into()
}
