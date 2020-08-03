###################################################################################
## Save Data ##
###################################################################################
## Make it sure that IP address is correctly set
- Open the 'Data_Manager/data_protocol.h'
- IP_ADDR is the definition that you need to care

## Make a folder to save data files
Made a folder 'experiment_data' below the dynacore folder (not in DynaController folder)

## Run the Status_Display
- After building statust_display, run the executable file 
- Option) -v (verbose mode)/ -q (default, quiet mode) 

### Run the main control program
- Then the Status_Display program will display and save the registered data


###################################################################################
# Register Data
###################################################################################
- Include <Utils/DataManager.hpp>
- Call the RegisterData function

ex)
    DataManager::GetDataManager()->RegisterData(&Q_, DYN_VEC, "config", NUM_Q);
    (pointer of data, data type, file name, size of data)
