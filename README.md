## Sonar Context
* **Robust Imaging Sonar-based Place Recognition and Localization in Underwater Environments**
	* Submitted to ICRA 2023


* **What is Sonar Context and polar key?**
	* Sonar context is a global descriptor, which encodes geometric characteristics of underwater environments.
	* Polar key is a 1D vector collected of average values in each row direction of the sonar context.
	* The descriptor consists of coarse (polar key) and fine description (Sonar Context) for
efficient loop closure detection.
	* The descriptor is robust for rotational and translational differences by adaptive shifting and matching algorithms.
	 ![main_fig](https://user-images.githubusercontent.com/68933951/215500050-c1974c55-10e0-494b-8a0f-a6c9d0cd30dd.png)

* **Author**
	* **Hogyun Kim, Gilhwan Kang, Seungjun Ma, Seokhwan Jeong and Younggun Cho**  
	
	

## Overview of our method
* **Place Recognition & Pose Graph SLAM in Aracati 2017 Datasets**
	* We provide the code of place recognition and pose graph slam version, respectively. 
	![sonar_context](https://user-images.githubusercontent.com/68933951/201089338-ed06170f-0d81-44df-86e4-81a417588374.gif)
		
## Datasets
* **[HOLOOCEAN](https://holoocean.readthedocs.io/en/latest/usage/usage.html)**
	* [googledrive](https://drive.google.com/drive/folders/1tPEZzdvOCRTILfkeq2X0KLuOM3v53kQL?usp=sharing)
	
* **KRISO (KOREA RESEARCH INSTITUTE OF SHIPS AND OCEAN ENGINEERING)**
	* not public  
	
* **Aracati 2017**
	* [github](https://github.com/matheusbg8/aracati2017)

  
## How to use sonar context?
**0. Requirements** 
  * [minisam](https://minisam.readthedocs.io/)
    * C++ compile and installation
		* Python installation  


**1. Download sonar context**
<pre>
<code>
    git clone https://github.com/sparolab/sonar_context.git
</code>
</pre>  


**2. Generate sonar context and polar key**
  * Modify the **patch size** in the /generate/img2sc_and_pk.py.
	* Set your **input path** (rectangular image called encoded polar image) and **output paths** (sonar context and polar key). 
<pre>
<code>
    cd ~/your_workspace/generate/
    python3 img2sc_and_pk.py
</code>
</pre>  


* **Set a path of sonar context and polar key dataset.txt**
	* In 14, 15 line, you edit your sonar context and polar key folder path to input. 

<pre>
<code>
    cd ~/your_workspace/scripts/place_recognition/
    cd ~/your_workspace/scripts/pose_graph_slam/
    vi datasets.txt
</code>
</pre>  
	
	
* **Place recognition**
<pre>
<code>
    cd ~/your_workspace/Sonar_Context/scripts/place_recognition/
    python3 main.py dataset.txt	
</code>
</pre>  
	
	
* **Pose graph slam**
<pre>
<code>
    cd ~/your_workspace/Sonar_Context/scripts/pose_graph_slam/
    python3 main.py dataset.txt
</code>
</pre>  


## Cite Sonar Context
<pre>
<code>
@inproceedings{kim2018scan,
  title={Scan context: Egocentric spatial descriptor for place recognition within 3d point cloud map},
  author={Kim, Giseop and Kim, Ayoung},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4802--4809},
  year={2018},
  organization={IEEE}
}
</code>
</pre>  


<pre>
<code>
@inproceedings{potokar2022holoocean,
  title={Holoocean: An underwater robotics simulator},
  author={Potokar, Easton and Ashford, Spencer and Kaess, Michael and Mangelson, Joshua G},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)},
  pages={3040--3046},
  year={2022},
  organization={IEEE}
}
</code>
</pre>  


<pre>
<code>
@article{dos2022cross,
  title={Cross-view and cross-domain underwater localization based on optical aerial and acoustic underwater images},
  author={Dos Santos, Matheus M and De Giacomo, Giovanni G and Drews-Jr, Paulo LJ and Botelho, Silvia SC},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={4969--4974},
  year={2022},
  publisher={IEEE}
}
</code>
</pre>  


## Contributor
* Hogyun Kim (12170550@inha.edu)
* Gilhwan Kang (22222151@inha.edu)
* Seungjun Ma (richard7714@inha.edu)
* Seokhwan Jeong (12171433@inha.edu)
* Younggun Cho (yg.cho@inha.ac.kr)  


## Supplementary
* **Supplementary materials are available at https://sites.google.com/view/sonarcontext**
