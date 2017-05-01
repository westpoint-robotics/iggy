Jupyter .bag Analysis setup instructions. Made by 2LT John Oberholtzer
Updated 8JUN2016


ABOUT:

"The Jupyter Notebook is a web application that allows you to create and share documents that contain live code, equations, visualizations and explanatory text. Uses include: data cleaning and transformation, numerical simulation, statistical modeling, machine learning and much more." - http://jupyter.org/

Essentially, We use Jupyter as a browswer based setup for python analysis of our .bag files by converting individual rostopics to .csv files and then processing and graphing the data.
Example: Running current code creates a lines3d_(datetimegroup).bag file. This gets converted into a .csv and from that the jupyter system creates a three dimensional graph of where the robot saw white lines.



INSTALLATION

Python needs to be installed if it is not already, as well as all the libraries utilized by TEST_MAIN.ipynb
All import statements used shown below:

import matplotlib
import csv
import sys
import ast
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from datetime import datetime
from struct import *
from mpl_toolkits.mplot3d import Axes3D

If Jupyter is not already installed, follow the instructions at http://jupyter.readthedocs.io/en/latest/install.html

#----- DML Installed jupynter with the command: sudo pip install jupyter
# ---- DML Needed to update matplotlib from the Ubuntu debault of 1.3 to 1.4+ by running:  sudo pip install --upgrade matplotlib

The installation through Anaconda works well enough. I discourage further use of conda however, as an attempt to install pandas using conda later resulted in many conflicts with ROS after changing some enviromental paths.

Once Jupyter is installed, it should be accessible by running:
jupyter notebook
Upon running this command, a web browser will open showing the local directory.


ORGANIZATION

All of the jupyter files are located under home/jupyter. This is not required; i created the folder myself. If you run jupyter from home you can easily navigate to any other folder, but putting everything in one place simplified things. The format I have adopted for individual runs is to for one set of bag files, create a folder called RUN_(DTG)
The Date-time-group (DTG) is the suffix that is automatically appended by the rosbag_record.launch file that is run by igvc_run_everything. For example, if you see one of the bag files from your most recent run comes back as imu_raw_2016-05-26-24-32.bag, the "suffix" is "2016-05-26-24-32". I would then name a folder RUN_2016-05-26-24-32, and all of the .bag files from that run would go into that folder.

There are various miscellanous files around the jupyter folder from initial stages of putting things together. The important ones are TEST_MAIN.ipynb (the primary processing file) and bag2csv_v2, which converts .bag files to .csv files when they are in the correctly specified folder. bag2csv_v2 is a version of bag2csv.py (which is built to work on files in the same folder) which I modified to support having separate RUN_DTG directories for each run.



OPERATION

From a terminal, either in home or in home/jupyter, run the following command:
jupyter notebook

This will open up the web application automatically, with a file directory based in wherever you ran it from.
The directory navigation is pretty intuitive. Any files opened will create their own tabs in the browswer.
To get started, open TEST_MAIN.ipynb



WORKING WITH NOTEBOOKS

Jupyter notebooks have some interesting features that allow for selective execution of code. These 'notebooks' allow you to
organize code into Cells. You can add new cells or cut old ones out using buttons towards the top of the page.
In order to actually execute code, click the Cell tab on the top bar.
Here you will see six options: 
Run Cells, Run Cells and Select Below, Run Cells and Insert Below, Run All, Run all Above, and Run All Below

The ones I made use of were Run Cells, Run All, and Run All Above/Below.
Run Cells will run whatever cell you currently have selected. You can use this feature to run one cell at a time.
Run All will execute the entire page and is probably what you will use most if you aren't trying to ignore certain cells.
Run all above or run all below are exceptionally useful if part of the code does not need to be executed again. For example, if the .bag to .csv conversion has already happened, but you made modifications to the graphing code below that area, you can click in the cell below where the conversion happened and execute run all below. It will reference the same variables and files that were processed above. Note that this will not work if you have restarted jupyter and not executed the above code once already.

If you have TEST_MAIN.ipynb open, you will need to make a few edits before running.
There are comments in the file with similar instructions, but in the first cell you need to set the 'suffix' variable to the appropriate ending. You also ought to have a folder with the appropriate .bag files located therein. Once you have the RUN_(DTG) folder appropriately formatted, check the boolean list in cell 2. Each item in the list corresponds to a .bag file, and if you intend to process the .bag file then keep it as true. If the .bag file was not recorded or you do not wish to include it, edit the value to false. There was some error checking code in place but it's not working, feel free to fix it.

If any structural changes are made to how .bag files are recorded, and what topics are put into which .bag files, some moderate editing will need to be done to TEST_MAIN.ipynb. It is currently formatted to pull specific data from specific files, so it would mostly just be name replacement, but it is important to be aware of.


