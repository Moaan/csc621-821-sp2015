#ifndef __WINDREGISTRATION_H
#define __WINDREGISTRATION_H

#include "itkImageRegistrationMethod.h"
#include "itkAffineTransform.h"
#include "itkMutualInformationImageToImageMetric.h"
#include "itkGradientDescentOptimizer.h"
#include "itkNormalizeImageFilter.h"
#include "itkDiscreteGaussianImageFilter.h"
#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkCheckerBoardImageFilter.h"
#include "itkEllipseSpatialObject.h"
#include "itkSpatialObjectToImageFilter.h"
#include <stdlib.h>

template < typename ImageType, int Dimension >

class WindRegistration
{
		// Images are unsigned char pixel type but use floats internally 
		typedef   double                       InternalPixelType;
		typedef itk::Image< double, Dimension> InternalImageType;
		
		// Normalizes the images
		typedef itk::NormalizeImageFilter<ImageType, InternalImageType> NormalizeFilterType;
		
		// Smooths the images
		typedef itk::DiscreteGaussianImageFilter<InternalImageType,InternalImageType> GaussianFilterType;
		
		// Registration components 
		typedef itk::AffineTransform< double, Dimension > TransformType;
		typedef itk::GradientDescentOptimizer                  OptimizerType;
		typedef itk::LinearInterpolateImageFunction<
										InternalImageType,
										double             > InterpolatorType;
		typedef itk::ImageRegistrationMethod<
										InternalImageType,
										InternalImageType >  RegistrationType;
		typedef itk::MutualInformationImageToImageMetric<
											  InternalImageType,
											  InternalImageType >    MetricType;
											  
		typedef typename RegistrationType::ParametersType ParametersType;
		
		typedef itk::ResampleImageFilter< ImageType, ImageType >    ResampleFilterType;
		
		
	private:
			
		typename NormalizeFilterType::Pointer fixedNormalizer;
		typename NormalizeFilterType::Pointer movingNormalizer;

		typename GaussianFilterType::Pointer fixedSmoother;
		typename GaussianFilterType::Pointer movingSmoother;

		typename TransformType::Pointer      transform;
		typename OptimizerType::Pointer      optimizer;
		typename InterpolatorType::Pointer   interpolator;
		typename RegistrationType::Pointer   registration;	

		typename MetricType::Pointer         metric;
		
		ImageType *fixedImage, *movingImage;
	
	public:
		WindRegistration()
		{
			fixedNormalizer = NormalizeFilterType::New();
			movingNormalizer = NormalizeFilterType::New();
			
			fixedSmoother  = GaussianFilterType::New();
			movingSmoother = GaussianFilterType::New();

			transform     = TransformType::New();
			optimizer     = OptimizerType::New();
			interpolator  = InterpolatorType::New();
			registration  = RegistrationType::New();	
			
			metric        = MetricType::New();
			
			optimizer->SetLearningRate( 0.1 );
			optimizer->SetNumberOfIterations( 1000 );
			
		}
		
		void SetInputs(ImageType *fixedImage, ImageType *movingImage)
		{
			fixedNormalizer->SetInput(  fixedImage );
			movingNormalizer->SetInput( movingImage );
			
			this->fixedImage = fixedImage;
			this->movingImage = movingImage;
		}
		
		void SetLearningRate( double learningRate) 
		{
			optimizer->SetLearningRate( learningRate );
		}

		void SetNumberOfIterations( int numberOfIterations)
		{
			optimizer->SetNumberOfIterations( numberOfIterations );
		}
				
		ImageType* GetOutput() {

			fixedSmoother->SetVariance( 2.0 );
			movingSmoother->SetVariance( 2.0 );
		 
			fixedSmoother->SetInput( fixedNormalizer->GetOutput() );
			movingSmoother->SetInput( movingNormalizer->GetOutput() );

			registration->SetOptimizer(     optimizer     );
			registration->SetTransform(     transform     );
			registration->SetInterpolator(  interpolator  );


			registration->SetMetric( metric  );
			
			metric->SetFixedImageStandardDeviation(  0.4 );
			metric->SetMovingImageStandardDeviation( 0.4 );

			registration->SetFixedImage(    fixedSmoother->GetOutput()    );
			registration->SetMovingImage(   movingSmoother->GetOutput()   );
			
			fixedNormalizer->Update();
			typename ImageType::RegionType fixedImageRegion = fixedNormalizer->GetOutput()->GetBufferedRegion();//GetLargestPossibleRegion();
			registration->SetFixedImageRegion( fixedImageRegion );

			ParametersType initialParameters( transform->GetNumberOfParameters() );

			// rotation matrix (identity)
			initialParameters[0] = 1.0;  // R(0,0)
			initialParameters[1] = 0.0;  // R(0,1)
			initialParameters[2] = 0.0;  // R(1,0)
			initialParameters[3] = 1.0;  // R(1,1)

			// translation vector
			initialParameters[4] = 0.0;
			initialParameters[5] = 0.0;

			registration->SetInitialTransformParameters( initialParameters );
			
			const unsigned int numberOfPixels = fixedImageRegion.GetNumberOfPixels();

			const unsigned int numberOfSamples = static_cast< unsigned int >( numberOfPixels * 0.01 );

			metric->SetNumberOfSpatialSamples( numberOfSamples );

			//optimizer->SetLearningRate( 15.0 ); //"All the sampled point mapped to outside of the moving image"
			//optimizer->SetLearningRate( 1.0 );
			
			optimizer->MaximizeOn();
			
			try
			{
				registration->Update();
				/*std::cout << "Optimizer stop condition: "
						  << registration->GetOptimizer()->GetStopConditionDescription()
						  << std::endl;*/
			}
			catch( itk::ExceptionObject & err )
			{
				std::cout << "ExceptionObject caught !" << std::endl;
				std::cout << err << std::endl;
				exit(EXIT_FAILURE);		
			}

			ParametersType finalParameters = registration->GetLastTransformParameters();

			//std::cout << "Final Parameters: " << finalParameters << std::endl;

			unsigned int numberOfIterations = optimizer->GetCurrentIteration();

			double bestValue = optimizer->GetValue();

			// Print out results
			/*
			std::cout << std::endl;
			std::cout << "Result = " << std::endl;
			std::cout << " Iterations    = " << numberOfIterations << std::endl;
			std::cout << " Metric value  = " << bestValue          << std::endl;
			std::cout << " Numb. Samples = " << numberOfSamples    << std::endl;*/

			typename TransformType::Pointer finalTransform = TransformType::New();

			finalTransform->SetParameters( finalParameters );
			finalTransform->SetFixedParameters( transform->GetFixedParameters() );

			typename ResampleFilterType::Pointer resample = ResampleFilterType::New();

			resample->SetTransform( finalTransform );
			resample->SetInput( movingImage );

			resample->SetSize(    fixedImage->GetLargestPossibleRegion().GetSize() );
			resample->SetOutputOrigin(  fixedImage->GetOrigin() );
			resample->SetOutputSpacing( fixedImage->GetSpacing() );
			resample->SetOutputDirection( fixedImage->GetDirection() );
			resample->SetDefaultPixelValue( 100 );
			
			
			return resample->GetOutput();
		}

};
#endif