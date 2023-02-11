#pragma once

#include "Eigen/Eigen"

/*
A set of Eigen matrix utilities that implements proper support for resizing
matrix.
*/
class DynamicEigen
{

public:

    /*
    Simple reimplementation of "conservativeResize" from more recent versions of
    Eigen. Only usable for adding a single row to a matrix dynamic on the row
    axis but not the column axis.
    */
    template <typename T, int lockedDimensions>
    void addRow(Eigen::Matrix<T, Eigen::Dynamic, lockedDimensions>* &oldMatrix,
                                                const std::vector<T> &newEntry);

    /*
    Simple reimplementation of "conservativeResize" from more recent versions of
    Eigen. Only usable for adding a single row to a matrix dynamic on the row
    axis but not the column axis. This constant version will place a single
    value in every element of the new row.
    */
    template <typename T, int lockedDimensions>
    void addRow(Eigen::Matrix<T, Eigen::Dynamic, lockedDimensions>* &oldMatrix,
                                                                       T value);
};

/*
Simple reimplementation of "conservativeResize" from more recent versions of
Eigen. Only usable for adding a single row to a matrix dynamic on the row axis
but not the column axis. DESTROYS THE OLD MATRIX.
*/
template <typename T, int lockedDimensions>
void DynamicEigen::addRow(Eigen::Matrix<T, Eigen::Dynamic, lockedDimensions>*
                                     &oldMatrix, const std::vector<T> &newEntry)
{
    // Get the index of the last entry in the output matrix.
    int newIndex = oldMatrix->rows();

    // Create a new matrix with room for an extra entry.
    Eigen::Matrix<T, Eigen::Dynamic, lockedDimensions>* newMatrix = new
        Eigen::Matrix<T, Eigen::Dynamic, lockedDimensions>(oldMatrix->rows()+1,
                                                              lockedDimensions);

    // Set the new matrix up with the content of the old.
    if(oldMatrix->rows() > 0)
    {
        newMatrix->block(0, 0, oldMatrix->rows(), lockedDimensions) =
                                                                   (*oldMatrix);
    }

    // Add the content of the new entry.
    for(int entry=0; entry<lockedDimensions; ++entry)
        (*newMatrix)(newIndex, entry) = newEntry[entry];

    // Replace the old matrix with the new.
    free(oldMatrix);
    oldMatrix = newMatrix;
}

/*
Simple reimplementation of "conservativeResize" from more recent versions of
Eigen. Only usable for adding a single row to a matrix dynamic on the row
axis but not the column axis. This constant version will place a single
value in every element of the new row.
*/
template <typename T, int lockedDimensions>
void DynamicEigen::addRow(Eigen::Matrix<T, Eigen::Dynamic, lockedDimensions>*
                                                            &oldMatrix, T value)
{
   // Get the index of the last entry in the output matrix.
   int newIndex = oldMatrix->rows();

   // Create a new matrix with room for an extra entry.
   Eigen::Matrix<T, Eigen::Dynamic, lockedDimensions>* newMatrix = new
       Eigen::Matrix<T, Eigen::Dynamic, lockedDimensions>(oldMatrix->rows()+1,
                                                              lockedDimensions);

   // Set the new matrix up with the content of the old.
   if(oldMatrix->rows() > 0)
   {
       newMatrix->block(0, 0, oldMatrix->rows(), lockedDimensions) =
                                                                   (*oldMatrix);
   }

   // Add the content of the new entry.
   for(int entry=0; entry<lockedDimensions; ++entry)
       (*newMatrix)(newIndex, entry) = value;

   // Replace the old matrix with the new.
   free(oldMatrix);
   oldMatrix = newMatrix;
}
