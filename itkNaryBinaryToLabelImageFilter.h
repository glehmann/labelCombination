/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkNaryBinaryToLabelImageFilter.h,v $
  Language:  C++
  Date:      $Date: 2006/10/25 12:12:56 $
  Version:   $Revision: 1.33 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkNaryBinaryToLabelImageFilter_h
#define __itkNaryBinaryToLabelImageFilter_h

#include "itkNaryFunctorImageFilter.h"
#include "itkNumericTraits.h"

namespace itk
{
  
/** \class NaryBinaryToLabelImageFilter
 * \brief Implements an operator for pixel-wise addition of two images.
 *
 * This class is parametrized over the types of the two 
 * input images and the type of the output image. 
 * Numeric conversions (castings) are done by the C++ defaults.
 *
 * The pixel type of the input 1 image must have a valid defintion of
 * the operator+ with a pixel type of the image 2. This condition is 
 * required because internally this filter will perform the operation
 *
 *        pixel_from_image_1 + pixel_from_image_2
 *
 * Additionally the type resulting from the sum, will be cast to
 * the pixel type of the output image.
 * 
 * The total operation over one pixel will be
 *
 *  output_pixel = static_cast<OutputPixelType>( input1_pixel + input2_pixel )
 *
 * For example, this filter could be used directly for adding images whose
 * pixels are vectors of the same dimension, and to store the resulting vector
 * in an output image of vector pixels.
 *
 * \warning No numeric overflow checking is performed in this filter.
 *
 * \ingroup IntensityImageFilters  Multithreaded
 */

namespace Functor {  
  
template< class TInput, class TOutput >
class NaryBinaryToLabel
{
public:
  NaryBinaryToLabel() {}
  ~NaryBinaryToLabel() {}
  inline TOutput operator()( const std::vector< TInput > & B)
  {
    TOutput v = NumericTraits< TOutput >::Zero;
    TOutput ret = m_BackgroundValue;

    for( int i=0; i<B.size(); i++ )
      {
      // avoid the background value
      if( v == m_BackgroundValue )
        {
        v++;
        }

      if( B[i] == m_ForegroundValue )
        {
        ret = v;
        }

      v++;
      }

    return ret;
  }

  bool operator!= (const NaryBinaryToLabel& n) const
  {
    return n.m_BackgroundValue != m_BackgroundValue || n.m_ForegroundValue != m_ForegroundValue;
  }

  TOutput m_BackgroundValue;
  TInput m_ForegroundValue;
}; 
}
template <class TInputImage, class TOutputImage>
class ITK_EXPORT NaryBinaryToLabelImageFilter :
    public
NaryFunctorImageFilter<TInputImage,TOutputImage, 
                       Functor::NaryBinaryToLabel<typename TInputImage::PixelType, typename TOutputImage::PixelType > > 
{
public:
  /** Standard class typedefs. */
  typedef NaryBinaryToLabelImageFilter  Self;
  typedef NaryFunctorImageFilter<TInputImage,TOutputImage, 
                                 Functor::NaryBinaryToLabel<typename TInputImage::PixelType, typename TOutputImage::PixelType > >  Superclass;
  typedef SmartPointer<Self>   Pointer;
  typedef SmartPointer<const Self>  ConstPointer;

  /** Some convenient typedefs. */
  typedef TInputImage InputImageType;
  typedef typename InputImageType::Pointer         InputImagePointer;
  typedef typename InputImageType::ConstPointer    InputImageConstPointer;
  typedef typename InputImageType::RegionType      InputImageRegionType;
  typedef typename InputImageType::PixelType       InputImagePixelType;

  typedef TOutputImage OutputImageType;
  typedef typename OutputImageType::Pointer         OutputImagePointer;
  typedef typename OutputImageType::ConstPointer    OutputImageConstPointer;
  typedef typename OutputImageType::RegionType      OutputImageRegionType;
  typedef typename OutputImageType::PixelType       OutputImagePixelType;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro(InputConvertibleToOutputCheck,
    (Concept::Convertible<typename TInputImage::PixelType,
                          typename TOutputImage::PixelType>));
  itkConceptMacro(InputHasZeroCheck,
    (Concept::HasZero<typename TInputImage::PixelType>));
  /** End concept checking */
#endif

  /**
   * Set/Get the value used as "foreground" in the input images.
   * Defaults to NumericTraits<PixelType>::max().
   */
  itkSetMacro(ForegroundValue, InputImagePixelType);
  itkGetConstMacro(ForegroundValue, InputImagePixelType);

  /**
   * Set/Get the value used as "background" in the output image.
   * Defaults to NumericTraits<PixelType>::Zero.
   */
  itkSetMacro(BackgroundValue, OutputImagePixelType);
  itkGetConstMacro(BackgroundValue, OutputImagePixelType);

protected:
  NaryBinaryToLabelImageFilter()
    {
    m_BackgroundValue = NumericTraits< InputImagePixelType >::Zero;
    m_ForegroundValue = NumericTraits< InputImagePixelType >::max();
    }

  virtual ~NaryBinaryToLabelImageFilter() {}

  void GenerateData()
    {
    this->GetFunctor().m_BackgroundValue = m_BackgroundValue;
    this->GetFunctor().m_ForegroundValue = m_ForegroundValue;
    Superclass::GenerateData();
    }

private:
  NaryBinaryToLabelImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  OutputImagePixelType m_BackgroundValue;
  InputImagePixelType m_ForegroundValue;
};

} // end namespace itk


#endif
