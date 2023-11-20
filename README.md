# as2_platform_template

Template for Aerostack2 Nodes

## API
For documentation, use [Doxygen](https://www.doxygen.nl/index.html). The documentation files have to be built locally:

1. Install doxygen:
    ```
    sudo apt install doxygen
    ```
2. Run doxygen in the root folder `as2_platform_template/` by:
    ```
    doxygen Doxyfile
    ```
3. Open the documentation in by opening `as2_platform_template/doxygen/html/index.html`

## Building the package

Go to the root folder of the workspace and run:
```
colcon build --packages-select as2_platform_template
```

## Running colcon test

Run colcon test in by:
```
colcon test --packages-select as2_platform_template
```
   
Optional, add verbosity with `--event-handlers console_direct`

## References

M. Fernandez-Cortizas, M. Molina, P. Arias-Perez, R. Perez-Segui, D. Perez-Saura, and P. Campoy, 2023,  ["Aerostack2: A software framework for developing multi-robot aerial systems"](https://arxiv.org/abs/2303.18237), ArXiv DOI 2303.18237.
