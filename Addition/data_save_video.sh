#! /bin/bash
PATH_PREFIX="/home"
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "MAC OS detected"
    PATH_PREFIX="/Users"
elif [[ "$OSTYPE" == "linux"*  ]]; then
    echo "LINUX OS detected
    PATH_PREFIX="/home"
else
    echo "Unsupported OS. Cannot run script""
fi
echo $PATH_PREFIX
PATH_PACKAGE="$PATH_PREFIX/$USER/Repository/dynacore"
echo $PATH_PACKAGE

target_folder="$PATH_PREFIX/$USER/MyCloud/Mercury_Test_2018_05"
data_location=$PATH_PACKAGE

if [[ -z "${LATEST_FOLDER_NAME}" ]]; then
	echo "Error. The timestamped data folder does not exist. Make sure to source data_save_nas.sh first"
else
	echo "OK! The timestamped data folder environment name exists"

    mp4array=(`find ${data_location}/experiment_data/ -maxdepth 1 -name "*.mp4"`)
    if [ ${#mp4array[@]} -gt 0 ]; then 
        echo "OK! The mp4 files exist. Preparing to copy the files if it is possible"
	    echo " "
		if ! [[ `lsof ${data_location}/experiment_data/*.mp4 | grep python` ]] ; then # Check if we can safely access the file
			echo " "			
			echo "(ignore the lsof warnings)"
			echo "Copying mp4 files..."
			cp ${data_location}/experiment_data/*.mp4 ${target_folder}/${LATEST_FOLDER_NAME}/	
			if [ $? -eq 0 ]; then # Check if previous command was successful
			    #echo "  Successfully copied the mp4 files!"
			    #echo "  Moving local copy to trash."		    
			    # Move files to trash
				#mv $PATH_PACKAGE/experiment_data/*.mp4 /home/$USER/.local/share/Trash/files/
				rm $PATH_PACKAGE/experiment_data/*.mp4
                # Unset Folder name
				unset LATEST_FOLDER_NAME
			else
			    echo "Error. Could not copy the mp4 files"
			fi

		else
			echo " "
			echo "(ignore the lsof warnings)"			
		    echo "Error. The mp4 file is still being accessed by python"
		fi


	else
		echo "Error. No MP4 files exist"
	fi



fi

echo "...script finished."

