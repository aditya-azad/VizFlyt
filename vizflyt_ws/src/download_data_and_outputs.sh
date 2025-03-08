#!/bin/bash

# Define Google Drive Folder ID (Extracted from the provided link)
OUTPUT_ZIP_ID="1fmvDWYQrSjvT8ji8TZo-BJqHM8Nk7Ypj"
DATA_ZIP_ID="1oOK6Lwn9g86cvOHnWj_oNqpGzaPF6d37"

# Define Destination Directory
OUTPUT_DIR="vizflyt_viewer/outputs"
DATA_DIR="vizflyt_viewer/data"
DATA_ZIP="$DATA_DIR/data.zip"
OUTPUT_ZIP="$OUTPUT_DIR/outputs.zip"


# Create the output directory if it doesn't exist
mkdir -p $OUTPUT_DIR
mkdir -p $DATA_DIR

echo "Downloading ZIP file into $OUTPUT_DIR ..."
gdown --remaining-ok --id $OUTPUT_ZIP_ID -O $OUTPUT_ZIP


echo "Downloading ZIP file into $DATA_DIR ..."
gdown --remaining-ok --id $DATA_ZIP_ID -O $DATA_ZIP

echo "Extracting outputs..."
unzip -o $OUTPUT_ZIP -d $OUTPUT_DIR

echo "Extracting data..."
unzip -o $DATA_ZIP -d $DATA_DIR

rm -f $DATA_ZIP
rm -f $OUTPUT_ZIP

echo "Download completed!"
