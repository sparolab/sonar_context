## Sonar Context
* **Robust Imaging Sonar-based Place Recognition and Localization in Underwater Environments**
	* Submitted to ICRA 2023
	* Our paper is available at [arxiv](https://arxiv.org/abs/2305.14773)

* **What is Sonar Context and polar key?**
	* Sonar context is a global descriptor, which encodes geometric characteristics of underwater environments.
	* Polar key is a 1D vector collected of average values in each row direction of the sonar context.
	* The descriptor consists of coarse (polar key) and fine description (Sonar Context) for
efficient loop closure detection.
	* The descriptor is robust for rotational and translational differences by adaptive shifting and matching algorithms.
	 ![main_fig](https://user-images.githubusercontent.com/68933951/215500050-c1974c55-10e0-494b-8a0f-a6c9d0cd30dd.png)

* **Author & Contributor**
	* Hogyun Kim, Gilhwan Kang, Seungjun Ma, Seokhwan Jeong and Younggun Cho  
	
	

## Overview of our method
* **Place Recognition & Pose Graph SLAM in Aracati 2017 Datasets**
![sonar_context](https://user-images.githubusercontent.com/68933951/201089338-ed06170f-0d81-44df-86e4-81a417588374.gif)
		
## Datasets
![datasets-1 (1)](https://user-images.githubusercontent.com/68933951/215676213-8672d0df-17d5-4fd5-98b6-b8454543dd50.png)

* **[HOLOOCEAN](https://holoocean.readthedocs.io/en/latest/usage/usage.html)**
	* [googledrive](https://drive.google.com/drive/folders/1tPEZzdvOCRTILfkeq2X0KLuOM3v53kQL?usp=sharing)
	
* **KRISO (KOREA RESEARCH INSTITUTE OF SHIPS AND OCEAN ENGINEERING)**
	* not public  
	
* **Aracati 2017**
	* [github](https://github.com/matheusbg8/aracati2017)

  
## How to use sonar context?
**0. Download sonar context**
<pre>
<code>
    $ git clone https://github.com/sparolab/sonar_context.git
</code>
</pre>  


**1. Requirements** 
  * We implement our place recognition method in python3. 
<pre>
<code>
    $ pip3 install -r requirements.txt
</code>
</pre>  


**2. Generate sonar context and polar key**
  * Modify the **patch size** parameters to fit your datasets in the img2scpk.py.
	* Set your **input path** (rectangular image called encoded polar image) and **output paths** (sonar context and polar key). 
<pre>
<code>
    $ cd sonar_context/generate/
    $ python3 img2scpk.py
</code>
</pre>  

**3. Place Recognition and Localization**
* Create your **datasets.txt file** similar to examples.txt and modify the parameters to fit your datasets.

<pre>
<code>
    $ cd sonar_context/place_recognition/
    $ python3 main holoocean.txt
</code>
</pre>  
	

## Contact
* Hogyun Kim (hg.kim@inha.edu)


## Supplementary
* **[Submission video](https://www.youtube.com/watch?v=JRD_xuqtHZU)**

## Cite Sonar Context
<pre>
<code>
@article{kim2023robust,
  title={Robust Imaging Sonar-based Place Recognition and Localization in Underwater Environments},
  author={Kim, Hogyun and Kang, Gilhwan and Jeong, Seokhwan and Ma, Seungjun and Cho, Younggun},
  journal={arXiv preprint arXiv:2305.14773},
  year={2023}
}
</code>
</pre>  


## Thanks Sonar Context
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
@inproceedings{jang2021multi,
  title={Multi-session underwater pose-graph slam using inter-session opti-acoustic two-view factor},
  author={Jang, Hyesu and Yoon, Sungho and Kim, Ayoung},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={11668--11674},
  year={2021},
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
