fn main() {
    println!("cargo:rerun-if-changed=nrf52833_chip_layout.ld");
    println!("cargo:rerun-if-changed=../kernel_layout.ld");
}
