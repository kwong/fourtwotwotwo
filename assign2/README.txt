Preliminaries
-------------
Code for TelosB Node is stored in /TelosBPart 
Code for SBT80 Node is stored in /SBT80Part
 

Instructions
-------------
1. In a terminal, point the current directory to /TelosBPart before executing 'make telosb'
2. Point the current directory to /SBT80Part before executing 'make telosb'
2. Create a new job on Indriya, with SerialToRadioMsg.class as the class file and "main.exe"
   in the /build folder of both /TelosBPart and /SBT80Part as the program files.
3. After selecting a pair of Nodes (TelosB&SBT80) and scheduling the job. First send an initiation command to the TelosB mote by running the following command:
        ./send.py indriya.comp.nus.edu.sg [port_for_TelosB_mote] 
4. You may then view the logged data by running the following command:
       For TelosB: ./receive.py indriya.comp.nus.edu.sg [port_for_TelosB_mote]
       For SBT80 : ./receive.py indriya.comp.nus.edu.sg [port_for_SBT80_mote]



Contact me @
Email: kangwei@nus.edu.sg



