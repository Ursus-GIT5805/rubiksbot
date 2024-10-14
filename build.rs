fn main() {
	if let Ok(path) = std::env::var("OPENCV_PATH") {
		println!("cargo::rustc-link-arg=-Wl,-rpath-link,{}", path);
		println!("cargo::rustc-link-search=all={}", path);
	}
}
