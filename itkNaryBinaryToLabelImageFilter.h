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
 *
 * \brief Combine several binary images in a single labeled image
 *
 * The objects from the first image get the first label, the objects from the second image
 * get the second one, etc.
 * Only the pixels with the value set with SetForegroundValue(InputImageType) are considered
 * to be an object. The other are background.
 * The SetIgnoreCollision(bool) method let the user choose to ignore the
 * labeled region collision or not. By default, they are ignored.
 * The SetBackgroundValue(OutputPixelType) let the user set the
 * value of the background label.
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
    bool valueFound = false;

    for( int i=0; i<B.size(); i++ )
      {
      // avoid the background value
      if( v == m_BackgroundValue )
        {
        v++;
        }

      if( B[i] == m_ForegroundValue )
        {
        if( !m_IgnoreCollision && valueFound )
          {
          itkGenericExceptionMacro( << "Label collision detected." );
          }
        ret = v;
        valueFound = true;
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
  bool m_IgnoreCollision;
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

  itkSetMacro(IgnoreCollision, bool);
  itkGetConstMacro(IgnoreCollision, bool);
  itkBooleanMacro(IgnoreCollision);

protected:
  NaryBinaryToLabelImageFilter()
    {
    m_BackgroundValue = NumericTraits< InputImagePixelType >::Zero;
    m_ForegroundValue = NumericTraits< InputImagePixelType >::max();
    m_IgnoreCollision = true;
    }

  virtual ~NaryBinaryToLabelImageFilter() {}

  void GenerateData()
    {
    this->GetFunctor().m_BackgroundValue = m_BackgroundValue;
    this->GetFunctor().m_ForegroundValue = m_ForegroundValue;
    this->GetFunctor().m_IgnoreCollision = m_IgnoreCollision;
    Superclass::GenerateData();
    }

  void PrintSelf( std::ostream& os, Indent indent) const
    {
    Superclass::PrintSelf( os, indent );
    os << indent << "Background Value: " << static_cast<typename NumericTraits<OutputImagePixelType>::PrintType>(m_BackgroundValue) << std::endl;
    os << indent << "Foreground Value: " << static_cast<typename NumericTraits<InputImagePixelType>::PrintType>(m_ForegroundValue) << std::endl;
    os << indent << "Ignore Collision: " << static_cast<typename NumericTraits<bool>::PrintType>(m_IgnoreCollision) << std::endl;
    }


private:
  NaryBinaryToLabelImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  OutputImagePixelType m_BackgroundValue;
  InputImagePixelType m_ForegroundValue;
  bool m_IgnoreCollision;
};

} // end namespace itk


#endif
