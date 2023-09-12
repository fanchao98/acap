# As-continuous-as-possible Extrusion-based Fabrication of Surface Models [[Project page]](https://fanchao98.github.io/ACAP%20page/acap.html)
![.](/teaser.png)
In this study, we propose a computational framework for optimizing the continuity of the toolpath in fabricating surface models on an extrusionbased 3D printer. We introduce a criterion called the “one-path patch” (OPP) to represent a patch on the surface of the shell that can be
traversed along one path by considering the constraints on fabrication. We study the properties of the OPPs and their merging operations to propose a bottom-up OPP merging procedure to decompose the given shell surface into a minimal number of OPPs, and to generate the "as-continuous-aspossible" (ACAP) toolpath.

# Dependency
You should install **CGAL**, **opencv**, **boost(1_76_0)**, **eigen3**, [**Curvislicer**](https://github.com/mfx-inria/curvislicer) before compiling this project.

# Citation
If you use the code for your research, please cite the paper:
```
@article{zhong2023continuous,
  title={As-continuous-as-possible Extrusion-based Fabrication of Surface Models},
  author={Zhong, Fanchao and Xu, Yonglai and Zhao, Haisen and Lu, Lin},
  journal={ACM Transactions on Graphics},
  volume={42},
  number={3},
  pages={1--16},
  year={2023},
  publisher={ACM New York, NY}
}
```

# License
All rights about the program are reserved by the authors of this project. The programs can only be used for research purpose. In no event shall the author be liable to any party for direct, indirect, special, incidental, or consequential damage arising out of the use of this program.
