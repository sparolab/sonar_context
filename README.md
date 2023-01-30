## Sonar_Context
* **Robust Imaging Sonar-based Place Recognition and Localization in Underwater Environments**
	* ICRA 2023 under review


![sonar_context](https://user-images.githubusercontent.com/68933951/201089338-ed06170f-0d81-44df-86e4-81a417588374.gif)


* **Author**
	* **Hogyun Kim, Gilhwan Kang, Seungjun Ma, Seokhwan Jeong and Younggun Cho**
	
	
## Requirements
* **minisam**
	* Please read this page, **https://minisam.readthedocs.io/**
		* C++ compile and installation
		* Python installation
		
		
## Datasets
* **HOLOOCEAN**
	* HOLOOCEAN is available at **https://holoocean.readthedocs.io/en/latest/usage/usage.html**  
	
* **KRISO (KOREA RESEARCH INSTITUTE OF SHIPS AND OCEAN ENGINEERING)**
	* not public  
	
* **Aracati 2017**
	* Aracati 2017 is available at **https://github.com/matheusbg8/aracati2017**  
	
## How to use sonar context?
* **Download sonar context**
	* In your workspace,
<pre>
<code>
    git clone https://github.com/hogyun2/Sonar_Context.git
</code>
</pre>  

* **Set a path of image folder**
	* In 14 line of code, you put your image folder path to input. 
	* In 59, 60 line of code, you put your image folder path to output. 
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

## Contact
* Hogyun Kim (12170550@inha.edu)
* Gilhwan Kang (22222151@inha.edu)
* Seungjun Ma (12171780@inha.edu)
* Seokhwan Jeong (12171433@inha.edu)
* Younggun Cho (yg.cho@inha.ac.kr)  

## Supplementary
* **Supplementary materials are available at https://sites.google.com/view/sonarcontext**
