use std::{path::Path, fs};

const CONFIG_PATH: &'static str = "lib_ikfast.config";
const LIB_DIR: &'static str = "lib_ikfast";

pub struct IkFastConfig {
    active: bool,
    link_lib: Option<String>,
}

impl IkFastConfig {
    pub fn load(path: &str) -> Result<Self, String> {
        let raw = fs::read_to_string(path).expect(&format!("Failed to read from {path}"));

        let mut this = IkFastConfig {
            active: true,
            link_lib: None,
        };

        for line in raw.split('\n') {
            this.process_line(line.trim())?
        }

        Ok(this)
    }

    fn process_line(&mut self, line: &str) -> Result<(), String>{
        if line.is_empty() || line.starts_with('#') {
            return Ok(())
        }

        match line.split_once('=') {
            Some((key, value)) => match key.trim() {
                "active" => self.set_active(value.trim()),
                "link_lib" => { self.link_lib = Some(value.trim().to_owned()); Ok(()) },
                _ => Ok(()),
            },
            None => Err(format!("Line '{}' is not in the form `key=value`", line)),
        }
    }

    fn set_active(&mut self, value: &str) -> Result<(), String> {
        match value {
            "true" => self.active = true,
            "false" => self.active = false,
            s => return Err(format!("Invalid value for key 'active', '{s}'. Should be (true|false)")),
        }

        Ok(())
    }
}

fn main() -> Result<(), String> {
    if Path::new(CONFIG_PATH).is_file() {
        let config = IkFastConfig::load(CONFIG_PATH)?;

        if let (true, Some(link_lib)) = (config.active, config.link_lib) {
            eprintln!("[INFO]\tLinking ikfast");

            println!("cargo:rustc-link-search={LIB_DIR}");
            println!("cargo:rustc-link-lib=static={link_lib}");
            println!("cargo:rustc-cfg=link_ikfast");

            let name_path = format!("{LIB_DIR}/name.rs");

            fs::write(&name_path, format!("{:?}", link_lib)).expect(&format!("Failed to log library name to {name_path}"));

            return Ok(());
        }
    }

    eprintln!("[INFO]\tNot linking ikfast");

    Ok(())
}
