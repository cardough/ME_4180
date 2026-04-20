
# ME_4180

Assignments and projects relating to ME 4180 - Mechanical Considerations in Robotics

## Installation Instructions

1. Ensure Git (<https://git-scm.com/install/>) and Python 3.10 (<https://www.python.org/downloads/>) are installed on the system. Make sure their commands are added to the global Windows "Path" variable with highest priority, or the Mac/Linux equivalent.
2. Next, find a free folder to place the code in; ideally one that is not automatically backed up with Onedrive or similar.
3. Open a git bash terminal in the folder (can use an IDE or on Windows, right click in the background of the folder and select "open git bash terminal"). Once you have it open, run:

    ```bash
    git clone https://github.com/cardough/ME_4180.git
    ```

    This will create a new folder named ME_4180.
4. Open the folder named ME 4180, and open a terminal (I use Powershell) in that location. Run:

    ```bash
    py -3.10 -m venv .venv
    ```

    This should create a new, grayed-out folder named .venv, which will contain the project's own version of Python and all the package dependencies. Open `.venv\pyvenv.cfg` and verify that the python version is 3.10.
5. Close and reopen the terminal, and (.venv) should be visible to the left of the terminal prompt. This indicates that the virtual environment is “active”. If it doesn't show up after restarting the terminal, run the OS-appropriate “Activate” script found in the virtual environment’s scripts folder. Once the env is active, run:

    ```bash
    pip install -e .
    ```

    This should take some time to collect remote dependencies and install them.
6. You should now be able run scripts from the project either using your IDE's run button or by running the script in the terminal. Try:

    ```bash
    .venv/Scripts/python.exe src/programming_assignment_2.py
    ```

    to test if the environment was set up correctly.
