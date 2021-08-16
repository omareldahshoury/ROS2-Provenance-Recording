# ROS2-Provenance-Recording
This code repo models a running ROS 2 system into its equivalent Prov format.

### Requirements
Before clone this repository make sure that you have ROS2 installed in your system and source it in the terminal.

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

### There are two methods to use the ROS2Prov:

I) Recommended and easier method:
To run the GUI:<br>
1) cd to the directory where `GUI_prov.py` is located
2) Run the below command in the shell:
<pre>
python3 GUI_prov.py
</pre>

II) Manual method:
To extract the data from the ROS system using the python file: <br>
1) cd to the directory where `ROS2-data-extraction-updated.py` is located
2) Run the below command in the shell:
<pre>
python3 ROS2-data-extraction-updated.py
</pre>
This file has a start-stop functionality. The data extraction starts once you run the file using the above mentioned command. To terminate the process, please press Ctrl+C. The data will be stored in a .csv file with the name "Data_storage.csv". This file can be found in the src folder of the repository.

To take a snapshot of the current system and to view it: <br>
1) cd to the directory where `single_instance_prov_generator.py` is located
2) Run the below command in the shell:
<pre>
python3 single_instance_prov_generator.py
</pre>
3) You can find the Prov Model files in the folder `ROS2Prov Model`
This file will take the information about the current running nodes, topics etc and generate the prov model based on it.



### Future possibility
Currently we can get a single time snapshot of the system i.e. the data for the current time and convert it into a provenance model. We also have the functionality to store the data with a start-stop button. We can extend this to a real-time system which take the data and converts it into a prov model till the stop button is pushed.
