# as2_mc

Aerostack 2 Modular Classes

This repository is a **Sandbox** for playing arround designs arround ROS2.

There is NO intention to supply any usable piece of software, the intention
is making designs (and alternative designs) to study how to carry out some
ideas about software architecture relative to systems based on ROS2.

## Compiling with ninja-build

Ninja is a fast compiling builder.

### Installing:

```bash
sudo apt install ninja-build
```

### Use:

Do:

```bash
export CMAKE_GENERATOR=Ninja
```

Or add it to .bashrc:

```bash
echo "export CMAKE_GENERATOR=Ninja" >> ~/.bashrc
source ~/.bashrc
```