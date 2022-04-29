fn main() {
    // The fact that the PACs include a memory.x but don't do this galls me
    // constantly:
    println!("cargo:rerun-if-changed=memory.x");
}
