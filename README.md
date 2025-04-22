# How to run this project

This project is designed for **mini Automated Guided Vehicles (miniAGV)**. The following commands allow you to set up, run, and generate documentation for the project.

## Setup and Running the Project

### First-Time Setup or After Modifying `src` Code

If you are running the project for the first time or have made changes to the `src` code, you can build and set up the project by executing:

```bash
./run.sh
```
This will build the necessary components and prepare the environment to run the miniAGV.

### Running the Project (After Initial Build)

If the project has already been built and you simply want to run it again, use the following command:

```bash
./just_run.sh
```

This will execute the project without rebuilding the source files.

## Documentation

### Generating and Viewing Documentation

To generate the Doxygen HTML documentation and view it, run the following command:

```bash
./open_docs.sh
```

This will build the documentation from the source code and open it in your default web browser.
