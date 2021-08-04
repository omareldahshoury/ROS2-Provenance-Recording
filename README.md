# ROS2-Provenance-Recording
The following code extracts ROS 2 Provenance related information and displays it in prov related format

src folder has the codes used:<br>
* `dummy_prov_model.py` -> A simple example of how a simple Prov model for a simple talker-listener ROS system would look like. This was used as reference when creating the ROS-Prov Modelling System. A Jupyter Notebook has also been included for a step by step explanation of implementation of the system `dummy_prov_model_ipynb.ipynb`
* `ROS2-data-extraction.py` -> Extracts basic info about the ROS system and stores it locally. This file uses Command line (terminal) arguments to extract and save informatiom
* `ROS2-data-extraction_old.py` -> Previous iteration of the code with the infinite loop and interrup exit
* `ROS2-data-extraction-updated.py` -> Extracts basic info about the ROS system using `rclpy` library
*  `ros_to_prov_parser.py` -> The program uses the save files extracted using `ROS2-data-extraction.py` to build the prov model
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
