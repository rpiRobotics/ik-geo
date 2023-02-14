use std::path::Path;

fn main() {
    if Path::new("lib").is_dir() {
        eprintln!("[INFO]\tLinking ikfast");

        println!("cargo:rustc-link-search=lib");
        println!("cargo:rustc-link-lib=static=kuka_kr30l16");
        println!("cargo:rustc-cfg=link_ikfast");
    }
    else {
        eprintln!("[INFO]\tNot linking ikfast");
    }
}
