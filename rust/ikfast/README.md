The code in this folder is part of ikfast by OpenRAVE and is licensed under the Apache License, Version 2.0 which can be found at:
    [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Modifcations to `kuka_kr30l16.cpp` are marked with `NOTE` in the source and were made for the following purposes in order.

- Reconcile the version difference between `kuka_kr30l16.cpp` and `ikfast.h`
- Allow interoperability with rust by exposing a function that is not name-mangled and has a simple interface
