#ifndef COVAR_H
#define COVAR_H

#include <stdio.h>
#include <gsl/gsl_statistics_double.h>
#include <gsl/gsl_matrix_double.h>
#include <vector>

/*
 *  Okay, so here's what we need
 *  1) A data structure to store the data points (vector?)
 *  2) A way of packaging the data points (struct of floats?)
 *  3) A method to push new data points onto the data structure
 *  
 *  So the above is an API towards collecting data. On the ROS side,
 *  1) As new messages from the ROS topic arrives, we need the 
 *     data_structure.push() to happen
 * 
 *  Next, we need a method for generating the covariance matrix. This 
 *  should happen after the data collection period has expired. What needs 
 *  to happen when we generate the covariance matrix? 
 *  1) Convert the data_structure into a gsl_matrix
 *  2) Don't forget to allocate another gsl_matrix which will hold the 
 *     resulting covariance matrix.
 *  3) Double for-loop to generate the variance-covariance value for each
 *     element of the matrix
 */

struct odom_struct
{
    double x;
    double y; 
    double z; 
    double yaw;
    double pitch;
    double roll;
};

class covar
{
    private:
        gsl_matrix *covar_mat;
        std::vector<odom_struct> *data_points;
    
    public:
        covar();
        
        ~covar()
        {
            delete covar_mat;
            covar_mat = 0;
            delete data_points;
            data_points = 0;
        }
        
        print_mat()
        {
            for (auto i = 0; i < covar_mat->size1; i++)
            {
                printf("\n");
                for (auto j = 0; j < covar_mat->size2; j++)
                {
                    printf("%f", covar_mat->data[C->size2*i+j]);
                    if(j < covar_mat->size2 - 1) printf(", ");
                }
            }
            printf("\n");
        }
}

#endif // COVAR_H