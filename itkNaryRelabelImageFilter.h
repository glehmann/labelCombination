/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkNaryRelabelImageFilter.h,v $
  Language:  C++
  Date:      $Date: 2006/10/25 12:12:57 $
  Version:   $Revision: 1.16 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkNaryRelabelImageFilter_h
#define __itkNaryRelabelImageFilter_h

#include "itkInPlaceImageFilter.h"
#include "itkImageIterator.h"
#include "itkArray.h"

namespace itk
{
  
/** \class NaryRelabelImageFilter
 * \brief Implements pixel-wise generic operation of Nth similar images.
 *
 * This class is parameterized over the types of the input images
 * and the type of the output image.  It is also parameterized by the
 * operation to be applied.  A Functor style is used to represent the
 * function.
 *
 * All the input images are of the same type.
 * 
 * \ingroup IntensityImageFilters   Multithreaded
 */

template <class TInputImage, class TOutputImage >
class ITK_EXPORT NaryRelabelImageFilter :
    public InPlaceImageFilter<TInputImage,TOutputImage> 

{
public:
  /** Standard class typedefs. */
  typedef NaryRelabelImageFilter  Self;
  typedef InPlaceImageFilter<TInputImage, TOutputImage>  Superclass;
  typedef SmartPointer<Self>   Pointer;
  typedef SmartPointer<const Self>  ConstPointer;
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(NaryRelabelImageFilter, InPlaceImageFilter);

  /** Some typedefs. */
  typedef TInputImage InputImageType;
  typedef typename InputImageType::Pointer      InputImagePointer;
  typedef typename InputImageType::RegionType   InputImageRegionType; 
  typedef typename InputImageType::PixelType    InputImagePixelType; 
  typedef TOutputImage OutputImageType;
  typedef typename OutputImageType::Pointer     OutputImagePointer;
  typedef typename OutputImageType::RegionType  OutputImageRegionType;
  typedef typename OutputImageType::PixelType   OutputImagePixelType;
  typedef std::vector< InputImagePixelType >    NaryArrayType; 
  
  /** ImageDimension constants */
  itkStaticConstMacro(
    InputImageDimension, unsigned int, TInputImage::ImageDimension);
  itkStaticConstMacro(
    OutputImageDimension, unsigned int, TOutputImage::ImageDimension);

#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro(SameDimensionCheck,
    (Concept::SameDimension<InputImageDimension, OutputImageDimension>));
  itkConceptMacro(OutputHasZeroCheck,
    (Concept::HasZero<OutputImagePixelType>));
  /** End concept checking */
#endif

  /**
   * Set/Get the value used as "background" in the images.
   * Defaults to NumericTraits<PixelType>::Zero.
   */
  itkSetMacro(BackgroundValue, InputImagePixelType);
  itkGetConstMacro(BackgroundValue, InputImagePixelType);

protected:
  NaryRelabelImageFilter();
  virtual ~NaryRelabelImageFilter() {};

  void GenerateData();

private:
  NaryRelabelImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented
  InputImagePixelType m_BackgroundValue;

};

} // end namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkNaryRelabelImageFilter.txx"
#endif

#endif
