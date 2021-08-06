# ROS2-Provenance-Recording
This code repo models a running ROS 2 system into its equivalent Prov format

### Description of Prov Model
The outlines relating to the Prov Model have been defined in the file `Prov_Model.md`. This file describes the basic outlay of the system and the approach used to model the system

### Files in `src` Folder 
* `dummy_prov_model.py` -> A simple example of how a simple Prov model for a simple talker-listener ROS system would look like. This was used as reference when creating the ROS-Prov Modelling System. A Jupyter Notebook has also been included for a step by step explanation of implementation of the system `dummy_prov_model_ipynb.ipynb`.
* `ROS2-data-extraction.py` -> Extracts basic info about the ROS system and stores it locally. This file uses Command line (terminal) arguments to extract and save informatiom
* `ROS2-data-extraction_old.py` -> Previous iteration of the code with the infinite loop and interrup exit
* `ROS2-data-extraction-updated.py` -> Extracts basic info about the ROS system using `rclpy` library. This solution has much better performance and is magnitudes quicker than `ROS2-data-extraction.py`
* `short-intro-to-rclpy.py` -> A simple walkthrough some of the functions of the `rclpy` library. Read this if you are trying to understand the code used to extract the information using `rclpy`
* `ros_to_prov_parser.py` -> The program uses the save files extracted using `ROS2-data-extraction.py` to build the prov model
* `single_instance_prov_generator.py` -> This file takes a snapshot of the system using `rclpy` and then builds it's prov mode. Note that this program only creates the snapshot based on a single instance of the system. This is better in use cases where we'd like to get a picture of the current architecture of the ROS system.
* `prov_utilities.py` -> Classes containing the definitions of Prov Elements based on the descriptions specified in `Prov_Model.md`. This used the code from `ros_to_prov_parser.py` and edits were made to be in concordance with the current ROS Prov Model definition. 
* `GUI_prov.py` -> GUI Implementation which provides an interactive GUI for using the ROS2-Prov-Record
* `gui_config.py` -> Supplementary file for the GUI's visualizations


To download utilized libraries:<br>
1) `cd` to the directory where `requirements.txt` is located.
2) Optional: activate your virtualenv
3) Run the below command in the shell:
<pre>
pip install -r requirements.txt
</pre>

To run the GUI:<br>
1) cd to the directory where GUI_prov.py is located
2) Run the below command in the shell:
<pre>
python3 GUI_prov.py
</pre>
