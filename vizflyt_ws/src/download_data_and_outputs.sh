#!/bin/bash

# Define Google Drive Folder ID (Extracted from the provided link)
OUTPUT_FOLDER_ID="1_KvilYhkyaIswBAgXE1O6g5XPLdCuiLp"
DATA_ZIP_ID="18vlsAMRq08jshOF1BaD7JHsThib-L84O"

# Define Destination Directory
OUTPUT_DIR="vizflyt_viewer/outputs"
DATA_DIR="vizflyt_viewer/data"
DATA_ZIP="$DATA_DIR/data.zip"

# Create the output directory if it doesn't exist
mkdir -p $OUTPUT_DIR
mkdir -p $DATA_DIR

# Download entire folders using gdown with --remaining-ok to avoid errors
echo "Downloading folder into $OUTPUT_DIR ..."
gdown --folder --remaining-ok --id $OUTPUT_FOLDER_ID -O $OUTPUT_DIR

echo "Downloading ZIP file into $DATA_DIR ..."
gdown --remaining-ok --id $DATA_ZIP_ID -O $DATA_ZIP

echo "Extracting data..."
unzip -o $DATA_ZIP -d $DATA_DIR

rm -f $DATA_ZIP

echo "Download completed!"
