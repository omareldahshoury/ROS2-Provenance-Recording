# ROS2-Provenance-Recording
This code repo models a running ROS 2 system into its equivalent Prov format

### Description of Prov Model
The outlines relating to the Prov Model have been defined in the file `Prov_Model.md`. This file describes the basic outlay of the system and the approach used to model the system

### Setting up your system
First clone the repository to your computer
To download utilized libraries:<br>
1) `cd` to the directory where `requirements.txt` is located.
2) Optional: activate your virtualenv
3) Run the below command in the shell:
<pre>
pip install -r requirements.txt
</pre>

**Necessary:** *Launch your ROS system which you'd like to record*<br>

To extract the data from the ROS system using the python file: <br>
1) cd to the directory where `ROS2-data-extraction-updated.py` is located
2) Run the below command in the shell:
<pre>
python3 ROS2-data-extraction-updated.py
</pre>

To take a snapshot of the current system and to view it: <br>
1) cd to the directory where `single_instance_prov_generator.py` is located
2) Run the below command in the shell:
<pre>
python3 single_instance_prov_generator.py
</pre>
3) You can find the Prov Model files in the folder `ROS2Prov Model`

To run the GUI:<br>
1) cd to the directory where `GUI_prov.py` is located
2) Run the below command in the shell:
<pre>
python3 GUI_prov.py
</pre>
