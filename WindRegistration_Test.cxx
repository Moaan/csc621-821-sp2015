#include "WindRegistration.h"
#include "itkImageFileWriter.h"
#include "itkImageFileReader.h"
 
int main( int argc, char *argv[] )
{  
  if( argc < 3 )
  {
    std::cerr << "Missing Parameters.. " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " fixedImageFile  movingImageFile " << std::endl;
    return EXIT_FAILURE;
  }
  
  const    unsigned int    Dimension = 2;
  typedef  unsigned char   PixelType;

  typedef itk::Image< PixelType, Dimension >  ImageType;
  
  typedef itk::ImageFileReader< ImageType  >   FixedImageReaderType;
  typedef itk::ImageFileReader< ImageType >   MovingImageReaderType;
  FixedImageReaderType::Pointer   fixedImageReader     = FixedImageReaderType::New();
  MovingImageReaderType::Pointer  movingImageReader    = MovingImageReaderType::New();

  fixedImageReader->SetFileName(  argv[1] );
  movingImageReader->SetFileName( argv[2] ); 

  WindRegistration<ImageType, Dimension > registrator = WindRegistration<ImageType, Dimension >();  
  
  registrator.SetInputs(  fixedImageReader->GetOutput(), movingImageReader->GetOutput());

 
  registrator.SetLearningRate( 0.1 );
  registrator.SetNumberOfIterations( 1000 );
 
  
  typedef itk::ImageFileWriter< ImageType >  WriterType;
  WriterType::Pointer      writer =  WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput( registrator.GetOutput()   );
  writer->Update();
 
  return EXIT_SUCCESS;
}