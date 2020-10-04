# V3TL-SIMULATOR
A V2V Simulator is used for simulation of the paper https://iris.polito.it/retrieve/handle/11583/2822232/359602/COMSNET2020_finalVersion.pdf
For using, you need install SUMO, VEINS and OMNET++ in appropriate way.

After installing tools needed, if you are using Mac or Linux, run the bash file which is uploaded.

In omnetpp.ini file (in v2v_base/examples/veins/omnetpp.ini) you can tune the variables Nc is for number of cars in each group.

The results are stored in Results folder V3TLALGforpy, V3TLConfid.

V3TLALGforpy format: 1st column is for number of vehicles which participated in the scheduler. second is processing time of scheduler third is the number of times that scheduler runned and the last column is the number of levels that needs for clearing the intersection.

V3TLConfid format: 1st column is number of passing cars second column is simulation time 3d column is number of stop and goes and 4th is number of scheduled cars
