# Linear Subproblem Solutions (Rust)

## Usage

1. Clone the rust directory

2. Add the following to your project's `Cargo.toml`

    ```conf
    [dependencies.linear-subproblem-solutions-rust]
        path = ".../linear-subproblem-solutions/rust"
    ```

3. Add to your source files where needed

    ```rust
    use linear_subproblem_solutions_rust as subproblems;
    use subproblems::nalgebra as na;
    ```

### Running the Demo

See demo.rs for example usage.

```
$ cargo run
```

### Testing

#### Correctness Tests

```
$ cargo test correctness --release
```

For diagnostic info:

```
$ cargo test correctness --release -- --nocapture
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

1. Compile the generated file into a static library with `ikfast_proxy.cpp` and place it into a folder called `lib_ikfast` in the project directory.

    Example compilation for the msvc toolchain:

    ```
    $ cl /O2 /c /EHsc /Folib_ikfast/[NAME].obj /DIKFAST_NO_MAIN .../[NAME].cpp
    $ cl /O2 /c /EHsc /Folib_ikfast/ikfast_proxy.obj ikfast/ikfast_proxy.cpp
    $ lib lib_ikfast/[NAME].obj lib_ikfast/ikfast_proxy.obj /out:lib_ikfast/[NAME].lib
    ```

    Example compilation for the gnu toolchain:

    ```
    $ g++ -O3 -o lib_ikfast/[NAME].obj -DIKFAST_NO_MAIN .../[NAME].cpp
    $ g++ -O3 -o lib_ikfast/ikfast_proxy.obj ikfast/ikfast_proxy.cpp
    $ ar rcs lib_ikfast/[NAME].a lib_ikfast/kuka_kr30l16.obj lib_ikfast/ikfast_proxy.obj
    ```

    Make sure IKFAST_NO_MAIN is defined.

2. Create a file in the project directory called `lib_ikfast.config` and place the following contents into it.

    ```
    active = true
    link_lib = spherical_bot
    kinematics_h = -1*k, -1*j, -1*j, -1*i, -1*j, -1*i
    kinematics_p = 0, 0.35*i + 0.815*k, 1.2*k, 0.145*k + 1.545*i, 0, 0, 0.158*i
    ```

    `active` specifies whether or not to link.

    `link_lib` specifies the name of the library to link.

    `kinematics_*` is used to specify the kinematic parameters of the robot.
