# Linear Subproblem Solutions (Rust)

## Usage

1. Clone the rust directory

2. Add the following to your project's `Cargo.toml`

    ```rust
    [dependencies.linear-subproblem-solutions-rust]
        path = ".../linear-subproblem-solutions/rust"
    ```

3. Add to your source files where needed

    ```rust
    use linear_subproblem_solutions_rust as subproblems;
    use subproblems::nalgebra as na;
    ```

### Testing

#### Correctness Tests

```
$ cargo test correctness
```

For diagnostic info:

```
$ cargo test correctness -- --nocapture
```

#### Timing Tests

```
$ cargo test timing --release -- --nocapture --test-threads 1
```

### Benchmarking

```
$ cargo bench
```

### Linking ikfast

Compile the generated file into a static library with `ikfast_proxy.cpp` and place it into a folder called `lib_ikfast` in the project directory.

Example compilation for the msvc toolchain:

```
$ cl /O2 /c /EHsc /Folib_ikfast/[NAME].obj /DIKFAST_NO_MAIN .../[NAME].cpp
$ cl /O2 /c /EHsc /Folib_ikfast/ikfast_proxy.obj /DIKFAST_NO_MAIN ikfast/ikfast_proxy.cpp
$ lib lib_ikfast/[NAME].obj lib_ikfast/ikfast_proxy.obj /out:lib/[NAME].lib
```

Example compilation for the gnu toolchain:

```
$ g++ -O3 -o lib/[NAME].obj -DIKFAST_NO_MAIN .../[NAME].cpp
$ g++ -O3 -o lib/ikfast_proxy.obj -DIKFAST_NO_MAIN ikfast/ikfast_proxy.cpp
$ ar rcs lib/[NAME].a lib/kuka_kr30l16.obj lib/ikfast_proxy.obj
```
