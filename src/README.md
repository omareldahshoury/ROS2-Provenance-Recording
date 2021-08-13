# Brief Overview of the files in current Directory
### Files in `src` Folder 
* `dummy_prov_model.py` -> A simple example of how a simple Prov model for a simple talker-listener ROS system would look like. This was used as reference when creating the ROS-Prov Modelling System. A Jupyter Notebook has also been included for a step by step explanation of implementation of the system `dummy_prov_model_ipynb.ipynb`.
* `short-intro-to-rclpy.py` -> A simple walkthrough some of the functions of the `rclpy` library. Read this if you are trying to understand the code used to extract the information using `rclpy`
* `ROS2-data-extraction-updated.py` -> Extracts basic info about the ROS system using `rclpy` library and exports a pandas dataframe to a csv file. This solution has much better performance and is magnitudes quicker than `ROS2-data-extraction.py` which uses terminal commands.
* `single_instance_prov_generator.py` -> This file takes a snapshot of the system using `rclpy` and then builds it's prov model. **Note** that this program only creates the snapshot based on a single instance of the system. This is better in use cases where we'd like to get a picture of the current architecture of the ROS system.
* `prov_utilities.py` -> Classes containing the definitions of Prov Elements based on the descriptions specified in `Prov_Model.md`. This is based on the code from `ros_to_prov_parser.py`. 
* `GUI_prov.py` -> GUI Implementation which provides an interactive GUI for using the ROS2-Prov-Record
* `gui_config.py` -> Supplementary file for the GUI's visualizations

### Files in `src/old_src_discontinued` Folder
* `ROS2-data-extraction.py` -> Extracts basic info about the ROS system and stores it locally. This file uses Command line (terminal) arguments to extract and save informatiom
* `ROS2-data-extraction_old.py` -> Previous iteration of the code with the infinite loop and interrup exit
* `ros_to_prov_parser.py` -> The program uses the save files extracted using `ROS2-data-extraction.py` to build the prov model
