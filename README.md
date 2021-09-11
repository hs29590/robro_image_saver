
## Libraries to save images. 

### How to use dataCollection.launch

1. Clone [robro_cloud](https://github.com/hs29590/robro_cloud)
2. Install packages as mentioned in [Readme](https://github.com/hs29590/robro_cloud/blob/main/README.md) .
3. install robroCloud as mentioned in [Readme](https://github.com/hs29590/robro_cloud/blob/main/README.md) .
4. Create role and service account. and download JSON. 
https://docs.google.com/document/d/1WtwhxfyTrMaWV6cSIFOlwGeqxoYtF8q0I7Wr_UMoSdM/edit
5. Clone [robro_image_saver](https://github.com/hs29590/robro_image_saver)
6. Edit its [launch](https://github.com/hs29590/robro_image_saver/blob/main/launch/dataCollection.launch) file with required ros topic name and triggers to ignore etc.
7. Launch!
### Launch File Attributes
1. proj_name - Project Name [String]
2. storage_class - Storage Class for GCP Bucket [String]
3. storage_location - Storage Location for GCP Bucket [String]
4. image_topic_name - ROS topic for image grabing [String]
5. image_type - Format of file to be saved on GCP bucket [String]
6. camera_name - Name of camera from where image originated [String]
7. cred_path - Full path to credentials json file downloaded from GCP console [String]
8. image_size - Image resize values (width, height) [String] (optional)
9. crop_size - Image crop size (x,y,w,h) [String] (optional)
10. grayscale - Image grayscale or color [Boolean]
11. category - category of image [String] (optional)
12. category_topic_name - ROS topic for category [String] (optional)

Note - if one enters category topic name, script will take category from topic, else one can also specify category directly in launch file. If category is not required both parameters can be skipped.
