# Mobile Robotik

## Repository for the practical exercises of the lecture "Mobile Robotik".

### Authors

* [Tom Georgi](https://github.com/TomGeorgi) 
* [Florian Djokaj](https://github.com/Flobolo) 
* [David Wolpers](https://github.com/da721wol)

### Setup the virtual environment

1. Install virtualenv on __Linux__ via the pip command
   
        sudo -H pip3 install virtualenv

   and on __Windows__ via the pip command

        py 3 -m pip install virtualenv

1. Make sure you are in the root folder of this repository.
2. Create the virtualenv with the name `mobi` via the command
   
        virtualenv mobi

3. activate the virtual environment on __Linux__ with the command `source`.

        source mobi/bin/activate

   and on __Windows__ via the command `call`.

        call mobi/bin/activate

4. Now you can use pip from the virtual environment. Install the recommended packages.

        pip install numpy matplotlib

5. Open PyCharm and go to the settings (Key Combination: Ctrl + Alt + S). \
   Go to `Project: mobile-robotik` -> `Project Interpreter`. \

6. Add a new Project Interpreter
7. Select the Option `Virtual Environment` -> Enable `Existing environment`.
8.  Go to the location of the `mobi`-directory.
9.  Select the Python Interpreter, which is located in `mobi/bin/` with the name `python`.
10. Now you can run the python script.