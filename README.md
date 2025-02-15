# An occlusion-resistant circle detector using inscribed triangles (Pattern Recognition 2021)



## Introduction
This is the code for circle detection in images using inscribed triangles. Circle detection is a critical issue in pattern recognition and image analysis. Conventional methods such as Hough transform, suffer from cluttered backgrounds and concentric circles. We present a novel method for fast circle detection using inscribed triangles. The proposed algorithm is more robust against cluttered backgrounds, noise, and occlusion.



<table>
    <tr>
         <td ><center><img src="https://github.com/zikai1/CircleDetection/blob/master/input.png"> </center></td>
        <td ><center><img src="https://github.com/zikai1/CircleDetection/blob/master/det.png"> </center></td>
        <td ><center><img src="https://github.com/zikai1/CircleDetection/blob/master/det2.png"> </center></td>
    </tr>
</table>

**Please give a star and cite if you find this repo useful.**

## Instructions
### 1. Requirements
The code was implemented with VS 2019, OpenCV 3.4.7, and Eigen3.

### 2. Detection of your data
To test images for your own data. Run the 'test.cpp' in the './src' directory.  
It allows you to specify the input file path:  
cv::String path = "E:/Code/patterns/Images1/";  
and the output path for the detected results:  
cv::String dst = "E:/Code/patterns/result/";  
Here, you need to create two directories, ie, 'Images1' and 'result'. If there are corresponding ground truths (GT), then you can further add the GT path:  
cv::String GT = "E:/Code/patterns/GT/";

### 3. Data sets
Four real-world datasets for circle detection: Dataset Mini, Dataset GH, Dataset PCB, and Dataset MY, are provided. Dataset Mini contains 10 images which are used as a benchmark by several works. Dataset GH contains 258 gray real-world images. Dataset PCB contains 100 industrial printed circuit board images, which are also grayscale. Dataset MY contains 111 colorful real-world images. We also provide ground truths for each dataset.


## Suggestions
Due to the complexity of real-world images, we cannot hope a set of fixed parameters to get the best results for each image. To customize your purpose, we provide some suggestions:  
- The inlier ratio threshold 'T_inlier', the larger the more strict. Hence, to get more circles, you can slightly tune it down.
- The sharp angle threshold 'sharp_angle'. To detect small circles, you can slightly tune it up
- The other parameters are usually fixed.

## Citation
If you find our work useful in your research, please cite our paper:  
@article{zhao2021occlusion,
  title={An occlusion-resistant circle detector using inscribed triangles},
  author={Zhao, Mingyang and Jia, Xiaohong and Yan, Dong-Ming},
  journal={Pattern Recognition},
  volume={109},
  pages={107588},
  year={2021},
  publisher={Elsevier}
}

