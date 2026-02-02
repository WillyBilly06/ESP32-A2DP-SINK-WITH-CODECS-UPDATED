# Arduino Library Conversion Tool

The Arduino Library format has the following requirements:
- The source code (cpp and h files) are direcly in the src subdirectory
- In the src directory we can have additional sudirectories - but the referring #include command need to contain the subdirectory as well   

The setup.py is a custom Python tool which is used 

- to download the fdk-aac project from github and 
- to change the directory structure, so that it is compliant with the Arduino Library structure.
- We also need to update all source files to make sure that the #include is pointing to a valid path by adding the directory names.
- The html documentation is generated with the help of doxygen if it does not exist.
- We apply some additinal patches to correct compile errors which are related to the Architecture of the microcontrollers

