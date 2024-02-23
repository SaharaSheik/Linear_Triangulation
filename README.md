# Linear_Triangulation
Reconstruction of a three-dimensional (3D) scene using information obtained from multiple two-dimensional (2D) images (In this exercise two images). The primary concept involves determining the 3D coordinates of points within a scene by intersecting their corresponding 2D projections captured from distinct viewpoints. 

#Introduction
In this assignment we were required to implement linear triangulation method.  Linear is used for the reconstruction of a three-dimensional (3D) scene using information obtained from multiple two-dimensional (2D) images (In this exercise two images). The primary concept involves determining the 3D coordinates of points within a scene by intersecting their corresponding 2D projections captured from distinct viewpoints. This method can be used 3D reconstruction applications
Description of each step:
	In this assignment we were required to implement linear triangulation method.  Linear is used for the reconstruction of a three-dimensional (3D) scene using information obtained from multiple two-dimensional (2D) images (In this exercise two images). The primary concept involves determining the 3D coordinates of points within a scene by intersecting their corresponding 2D projections captured from distinct viewpoints. This method can be used 3D reconstruction applications.
##Step 1- Feature Matching: This step was implemented by professor already but involves detection and aligning features (keypoints) within the images. Popular methods for feature alignment involve techniques such as SIFT (Scale-Invariant Feature Transform) or ORB (Oriented FAST and Rotated BRIEF).  The code had adopted detectORBFeatures.
##Step 2 - Fundamental matrix: This step was implemented by professor already. It was done using the estimateFundamentalMatrix. The Fundamental Matrix describes the geometric relationship between two camera views in a stereo vision system. After calculating the Fundamental Matrix, it becomes applicable in the process of linear triangulation for inferring the three-dimensional location of a point through the triangulation of its projections in the two images. This matrix plays a key role in establishing associations between points across distinct images and imparts essential constraints for the determination of three-dimensional structure.
##Step 3 - Estimating the camera projection matrices: This step was implemented by professor already. It was done using the relativeCameraPose which gives us R and t.  We were given value of intrinsic calibration matrix 𝐊. Since it was assumed that it was fixed and not changing between the two views, value of K is calculated by
K=(cam1+cam2)/2
P_1=K×[I,0]
P_2=K×[R,t]
##Step 4 - Linear Triangulation: For each matched pair of points (correspondences), use linear triangulation to estimate the 3D coordinates of the point. Given the matched points 
m_1= [x_1,y_1 ]↔m_1= [x_2,y_2 ]
and 3×4 camera matrices P_1  and P_2  which were calculated in the above step 
The linear triangulation can be formulated as solving a linear system of equations. Where:
A.X= 0
I created A by following the formula in the assignment sheet 
A=[(x_1 P_1^3T-P_1^1T@y_1 P_1^3T-P_1^2T@x_2 P_2^3T-P_2^1T@y_2 P_2^3T-P_2^2T )]
Here the P_i^j for i we have two values, i = 1 or 2 for camera matrix〖 P〗_1 or P_2. The j means the j^th row of e camera projection matrix. Knowing that P_1 and P_2  were 3×4 matrixes we have 4 rows in A and the columns are for example x_1 which is a scalar. P_1^3T  is 3rd row P_1 matrix.  The 3rd row of  3×4 matrix will be dimension 1×4. So row one for example will be scalar × a 1×4 matrix minus first row P_1 which is a 1×4 matrix.  So the result is a 1×4 matrix. We have for them stacked in A.  So A turned out to be 4×4 matrix.
##Step 5 - Singular Value Decomposition and Dehomogenization: We know that 
A=UDV^T
We can obtain V matrix by doing Singular Value Decomposition of matrix A that we have obtained above.  MatLab has built in library for this which I utilized:
[~,~,V] = svd(A)
V is a homogeneous 4×1 vector representing the 3D point M, corresponding to the matching image points m_1 and m_2 . To Dehomogenize is to obtain the final 3D coordinates of the point in the world coordinate system. Dehomogenization (determining the Euclidean coordinates) takes placed by dividing by the last coordinate. I did this using the following  lines of code:
    M_homogeneous_i = V(:,end);
    pntcloud(:,i) = M_homogeneous_i(1:3) / M_homogeneous_i(4);
##Step 6 - Repeat from Step 4 for all matching image points to create a 3D point cloud: I did this by placing all my code from steps 4 and 5 into a loop. The loop iterated from 1 to number of existing points in Pnts1 matrix. It then adds each i^th pntcloud to the i^th column of our pntclpud matrix. So the pntcloud are stores in a col by col basis ready for display.
##Step 7 - This was to Render the point cloud. Which the code was already provide.
Challenges Encountered
The main challenge for this project was understanding the resulting matrixes so that the matrix operations could have been handled properly during the pntcloud loop.  I did this by displaying the matrix sizes and making adjustments.
Also the P_i^j was supposed to have the j^th  row transposed.  Upon inspection the dimension I realized the transpose would cause the P_i^j becomes a 4×1 matrix and thus not work for us as the resulting A matrix would then becomes 16×1  instead of 4×4.
Results and Observations and Conclusions:
In this section you can see the results of the 4 different image folders following by the 3 images that each run created: 
	Image pair with matching points
	Rendered 3D point cloud that my implementation of the algorithm generates 
	Reference 3D point cloud generated by a built-in Matlab function called triangulate
I have also listed the SNR values for each image set below
Picture	SNR
Globe	85.7411
Newkuba	83.8084
Piano	86.1589
Playroom	81.8707

#SNT: the Signal-to-Noise Ratio (SNR). To understand SNT I studied what Signal and Noise refer to in context of Linear Triangulation:
#Signal: Essentially the information that comes from the correspondences between the 2D projections of the point in multiple images and the strength of signal refers to its usefulness of the information
#Noise: Essentially describes the undesired or random fluctuations and errors within measurements or correspondences. These errors can arise different factors, such as measurement errors, inaccuracies in feature detection, or imperfections in camera equipment. 
#SNR=(Noise Power)/(Signal Power)
#SNR is often represented in decibels as such:
#SNR(dB)=10×log_10⁡〖((Noise Power)/(Signal Power))〗
##So to summarize SNT denotes the relationship between the strength of the background noise. Since linear triangulation is utilized to determine the 3D coordinates of a point based on its projections in multiple images. The significance of SNR in this context is its role in ensuring the precision and dependability of the triangulation outcomes. The higher the SNT the better more precise our 3D coordinates which points to a more accurate and robust linear triangulation.  A higher signal means that the true location of the point is more distinguishable from any noise and errors present in the measurements.
##Enhancing the Signal-to-Noise Ratio (SNR) in linear triangulation may include employing better feature detection methods, fine-tuning camera calibration, or implementing strategies such as filtering or outlier rejection to mitigate the influence of noise. What I noticed during my experimentation was that the SNR was not fixed. The value would change from try to try but only slightly by 1 or 2 points.
Upon examining the images, I observed that those captured with a close-up perspective, such as the globe, exhibited improved visualization, likely resulting in less noise. Despite the similar Signal-to-Noise Ratio (SNR) across all three images, the playroom had the lowest SNR, presumably due to the multitude of objects present. It seemed that images with fewer elements, like the globe or Newkuba, provided a clearer representation of 3D points, while busier scenes with numerous objects resulted in a less coherent 3D point map.

