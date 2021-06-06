# ROS2-Provenance-Recording
The following code extracts ROS 2 Provenance related information and displays it in prov related format

src folder has the codes used:
ROS2-data-extraction.py -> extracts basic info about the ROS system and stores it locally
ROS2-data-extraction_old.py -> previous iteration of the code with the infinite loop and interrup exit
prov_gui.py -> Main file for the GUI
gui_config.py -> Supplementary file for the GUI's visualizations


To download utilized libraries:
1- cd to the directory where requirements.txt is located.
2- Optional: activate your virtualenv
3- Run the below command in the shell
pip install -r requirements.txt
