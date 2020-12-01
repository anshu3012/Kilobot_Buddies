/*
# The MIT License (MIT)
#
# Copyright (c) January, 2015 michael otte
# Copyright (c) April, 2020 michael otte, University of Maryland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
*/

#define TWOBYTE unsigned short
#define FOURBYTE unsigned int // u long int on 32 bit systems
#define LONG unsigned int     // ditto 

struct float_array;
typedef struct float_array float_array;


struct Bitmap;
typedef struct Bitmap Bitmap;

struct Image;
typedef struct Image Image;

struct float_array
{
    int rows, cols; // size of A
    int temp;       // a flag to help with memory management
    int t;          // a flag to allow for easy transposing
    float** A;
};

float_array* make_float_array(int rows, int cols)
{
    float_array* this_array = (float_array*)calloc(1, sizeof(float_array));

    this_array->rows = rows;
    this_array->cols = cols;
    this_array->temp = 0;
    this_array->t = 0;

    this_array->A = (float**)calloc(rows, sizeof(float*));
    int i;
    for(i = 0; i < rows; i++)
        this_array->A[i] = (float*)calloc(cols, sizeof(float));
    return this_array;
}

// adjusts the array A randomly, where each column's max adjustment is given by +/- the corresponding 
// value in R_max. Note A itself is changed (a new array is not returned)
float_array* adjust_array_randomly(float_array* A, float* R_max)
{
  int i,j;
  float** AA = A->A;
  int rows = A->rows;
  int cols = A->cols;

  for(i = 0; i < rows; i++)
    for(j = 0; j < cols; j++)
      AA[i][j] += R_max[j]*(1 - 2*(float)(rand() % (int)(101))/100); // adds random number on [-R_max ... R_max]

  return A;
}

void destroy_float_array(float_array* this_array)
{
  if(this_array == NULL)
    return;

  int rows = this_array->rows;

  int i;
  for(i = 0; i < rows; i++)
    free(this_array->A[i]);

  free(this_array->A);
  free(this_array);
}

// prints the float array, also destroys it if it is a temp
void print_float_array(float_array* this_array)
{
  int i, j;
  if(this_array == NULL)
  {
    printf("0\n");
    return;
  }

  int rows = this_array->rows;
  int cols = this_array->cols;

  if(this_array->t == 0)
  {
    for(i = 0; i < rows; i++)
    {
      for(j = 0; j < cols; j++)
        printf("%f ",this_array->A[i][j]);
      printf("\n");
    }
  }
  else // it is inverted
  {
    for(j = 0; j < cols; j++)
    {
      for(i = 0; i < rows; i++)
        printf("%f ",this_array->A[i][j]);
      printf("\n");
    }
  }

  if(this_array->temp == 1)
    destroy_float_array(this_array);
}




struct Bitmap
{
  // BMP header stuff
  TWOBYTE BMP_Type;               // 2 bytes
  FOURBYTE BMP_Size;              // 4 bytes
  FOURBYTE BMP_Reserved;          // 4 bytes
  FOURBYTE BMP_Offset;            // 4 bytes

  // DIB header stuff
  FOURBYTE DBI_Size;              // 4 bytes
  LONG DBI_Width;                 // 4 bytes
  LONG DBI_Height;                // 4 bytes
  TWOBYTE DBI_Colorplanes;        // 2 bytes
  TWOBYTE DBI_BitsPerPixel;       // 2 bytes
  FOURBYTE DBI_CompressionMethod; // 4 bytes
  FOURBYTE DBI_ImageSize;         // 4 bytes
  LONG DBI_HorizPixelsPerMeter;   // 4 bytes
  LONG DBI_VertPixelsPerMeter;    // 4 bytes
  FOURBYTE DBI_PaletteSize;       // 4 bytes
  FOURBYTE DBI_ImportantColors;   // 4 bytes

  unsigned long BPP;
  unsigned long width;
  unsigned long height;
  unsigned long size;
  unsigned char* Bitmap_Image;
  unsigned char* palette;
  unsigned int bps;
  unsigned int KompressionFormat;
};

Bitmap* make_Bitmap()
{
  Bitmap* Bmp = (Bitmap*)calloc(1, sizeof(Bitmap));
  Bmp->BPP=0;
  Bmp->width=0;
  Bmp->height=0;
  Bmp->Bitmap_Image = NULL;
  Bmp->palette = NULL;
  Bmp->size=0;
  Bmp->bps=0;
  Bmp->KompressionFormat=0;

  return Bmp;
}

void destroy_Bitmap(Bitmap* Bmp)
{
  if(Bmp == NULL)
    return;

  free(Bmp->Bitmap_Image);

  if(Bmp->palette != NULL)
    free(Bmp->palette);

  free(Bmp);
}

void print_Bitmap_info(Bitmap* B)
{
  if( B == NULL)
	  return;

  printf("BMP Header: \n");
  printf(" Type: %x \n", B->BMP_Type);
  printf(" Size: %u \n", (unsigned int)B->BMP_Size);
  printf(" Reserved: %u \n", (unsigned int)B->BMP_Reserved);
  printf(" Offset: %u \n", (unsigned int)B->BMP_Offset);

  printf("DIB Header: \n");
  printf(" Size: %u \n", (unsigned int)B->DBI_Size);
  printf(" Width: %u \n", (unsigned int)B->DBI_Width);
  printf(" Height: %u \n", (unsigned int)B->DBI_Height);
  printf(" Color Planes: %u \n", (unsigned int)B->DBI_Colorplanes);
  printf(" Bits Per Pixel: %u \n", (unsigned int)B->DBI_BitsPerPixel);
  printf(" Compression Method: %u \n", (unsigned int)B->DBI_CompressionMethod);
  printf(" Image Size: %u \n", (unsigned int)B->DBI_ImageSize);
  printf(" Horizontal Pixels Per Meter: %u \n", (unsigned int)B->DBI_HorizPixelsPerMeter);
  printf(" Vertical Pixels PEr Meter: %u \n", (unsigned int)B->DBI_VertPixelsPerMeter);
  printf(" Palette Size: %u \n", (unsigned int)B->DBI_PaletteSize);
  printf(" Important Colors: %u \n", (unsigned int)B->DBI_ImportantColors);
}


Bitmap*  load_Bitmap_from_file(const char *filename)
{
  Bitmap* B = make_Bitmap();

  FILE* inf = NULL;
  unsigned int ImageIdx = 0;
  unsigned char* Bitmap_Image = NULL;

  if(!filename)
  {
    printf("can't open file: %s\n", filename);
    return NULL;
  }
  else
  {
    inf = fopen(filename,"rb");
    if(!inf)
    {
      printf("can't open file: %s\n", filename);
      return NULL;
    }
  }

  // read in the 14 byte BMP header
  fread(&B->BMP_Type,sizeof(TWOBYTE),1,inf);
  fread(&B->BMP_Size,sizeof(FOURBYTE),1,inf);
  fread(&B->BMP_Reserved,sizeof(FOURBYTE),1,inf);
  fread(&B->BMP_Offset,sizeof(FOURBYTE),1,inf);

  if(ferror(inf))
  {
    printf("problem with file header \n");
    fclose(inf);
    return NULL;
  }

  // read in the 40 byte DIB Header, assuming V3 is used
  fread(&B->DBI_Size,sizeof(FOURBYTE),1,inf);
  if(B->DBI_Size != 40)
  {
    printf("cannot open file %s because it does not use DBI header V3 %u \n", filename, (unsigned int)(B->DBI_Size));
    fclose(inf);
    return NULL;
  }
  fread(&B->DBI_Width,sizeof(LONG),1,inf);
  fread(&B->DBI_Height,sizeof(LONG),1,inf);
  fread(&B->DBI_Colorplanes,sizeof(TWOBYTE),1,inf);
  fread(&B->DBI_BitsPerPixel,sizeof(TWOBYTE),1,inf);
  fread(&B->DBI_CompressionMethod,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_ImageSize,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_HorizPixelsPerMeter,sizeof(LONG),1,inf);
  fread(&B->DBI_VertPixelsPerMeter,sizeof(LONG),1,inf);
  fread(&B->DBI_PaletteSize,sizeof(FOURBYTE),1,inf);
  fread(&B->DBI_ImportantColors,sizeof(FOURBYTE),1,inf);

  if(ferror(inf))
  {
    printf("problem with info header \n");
    fclose(inf);
    return NULL;
  }

  // extract color palette
  if(B->DBI_BitsPerPixel == 24)
  {
    // don't need to
  }
  else if(B->DBI_BitsPerPixel == 8 || B->DBI_BitsPerPixel == 4 || B->DBI_BitsPerPixel == 1)
  {

    int colors_used = B->DBI_PaletteSize;
    if(colors_used == 0)
      colors_used = (int)pow((float)2,(int)B->DBI_BitsPerPixel);

    //printf(" colors used: %d \n", colors_used);

    unsigned char* color_map = (unsigned char*)calloc(colors_used*4, sizeof(unsigned char));
    fread(color_map,sizeof(unsigned char),colors_used*4,inf);

    B->palette = color_map;
  }
  else
    printf("This number of bits per pixel (%u) not implimented \n", B->DBI_BitsPerPixel);


  fseek(inf,B->BMP_Offset,SEEK_SET);
  if(ferror(inf))
  {
    printf("problem with 'bfOffBits' \n");
    fclose(inf);
    return 0;
  }


  if(B->DBI_ImageSize != 0)
  {
    Bitmap_Image = (unsigned char*)calloc(B->DBI_ImageSize, sizeof(unsigned char));
    fread(Bitmap_Image,B->DBI_ImageSize,1,inf);
  }

  if(B->BMP_Type != 0x4D42)
  {
    printf("problem with magic number \n");
    fclose(inf);
  }

  if(!Bitmap_Image)
  {
    free(Bitmap_Image);
    fclose(inf);
  }

  if(Bitmap_Image==NULL)
    fclose(inf);

  B->Bitmap_Image = Bitmap_Image;

  fclose(inf);

  return B;
}


struct Image
{
  float_array* Red;
  float_array* Green;
  float_array* Blue;
};

Image* make_Image(int rows, int cols)
{
  Image* I = (Image*)calloc(1, sizeof(Image));
  I->Red = make_float_array(rows,cols);
  I->Green = make_float_array(rows,cols);
  I->Blue = make_float_array(rows,cols);

  return I;
}

void destroy_Image(Image* I)
{
  if(I == NULL)
    return;

  destroy_float_array(I->Red);
  destroy_float_array(I->Green);
  destroy_float_array(I->Blue);
  free(I);
}

void print_Image(Image* I)
{
  if(I == NULL)
    return;

  printf("Red: \n");
  print_float_array(I->Red);

  printf("Green: \n");
  print_float_array(I->Green);

  printf("Blue: \n");
  print_float_array(I->Blue);
}

// converts the Bitmap struct into an Image struct
Image* convert_Bitmap_to_image(Bitmap* B)
{
  if(B == NULL)
    return NULL;

  if(B->DBI_CompressionMethod != 0)
  {
    printf("unsupported compression type \n");
    return NULL;
  }

  int rows = B->DBI_Height;
  int cols = B->DBI_Width;

  Image* I = make_Image(rows, cols);
  float** Red = I->Red->A;
  float** Blue = I->Blue->A;
  float** Green = I->Green->A;
  unsigned char* Data = B->Bitmap_Image;

  int bits_per_pixel = B->DBI_BitsPerPixel;

  if(bits_per_pixel == 24)
  {
    int i, j, k;
    int col_size_with_pad = 3*cols;
    if(4*(col_size_with_pad/4) != col_size_with_pad)
      col_size_with_pad = 4*(col_size_with_pad/4+1);

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      for(j = 0; j < cols; j++)
      {
        Blue[i][j]  = (float)Data[k]/255;
        k++;
        Green[i][j] = (float)Data[k]/255;
        k++;
        Red[i][j] = (float)Data[k]/255;
        k++;
      }
    }
    return I;
  }
  else if(bits_per_pixel == 8)
  {
    int i, j, k;
    int col_size_with_pad = cols;
    if(4*(col_size_with_pad/4) != col_size_with_pad)
      col_size_with_pad = 4*(col_size_with_pad/4+1);

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      for(j = 0; j < cols; j++)
      {
        Blue[i][j]  = (float)B->palette[Data[k]*4]/255;
        Green[i][j] = (float)B->palette[Data[k]*4+1]/255;
        Red[i][j] = (float)B->palette[Data[k]*4+2]/255;
        k++;
      }
    }
    return I;

  }
  else if(bits_per_pixel == 4)
  {
    int i, j, k;
    int col_size_with_pad = cols;

    if(8*(col_size_with_pad/8) != col_size_with_pad)
      col_size_with_pad = 8*(col_size_with_pad/8+1);

    col_size_with_pad = col_size_with_pad/2;

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      unsigned char this_char = (unsigned char)Data[k];
      int bit_num = -1;
      int this_bit;

      for(j = 0; j < cols; j++)
      {
        bit_num++;
        if(bit_num == 2)
        {
          bit_num = 0;
          k++;
          this_char = Data[k];
        }
        this_bit = this_char/16;
        this_char = this_char << 4;


        Blue[i][j]  = (float)B->palette[this_bit*4]/255;
        Green[i][j] = (float)B->palette[this_bit*4+1]/255;
        Red[i][j] = (float)B->palette[this_bit*4+2]/255;
      }
    }
    return I;

  }
  else if(bits_per_pixel == 1)
  {
    int i, j, k;
    int col_size_with_pad = cols;

    if(32*(col_size_with_pad/32) != col_size_with_pad)
      col_size_with_pad = 32*(col_size_with_pad/32+1);

    col_size_with_pad = col_size_with_pad/8;

    for(i = rows-1; i >= 0; i--)
    {
      k = (rows-i-1)*col_size_with_pad;
      unsigned char this_char = (unsigned char)Data[k];
      int bit_num = -1;
      int this_bit;

      for(j = 0; j < cols; j++)
      {
        bit_num++;
        if(bit_num == 8)
        {
          bit_num = 0;
          k++;
          this_char = Data[k];
        }
        this_bit = this_char/128;
        this_char = this_char << 1;


        Blue[i][j]  = (float)B->palette[this_bit*4]/255;
        Green[i][j] = (float)B->palette[this_bit*4+1]/255;
        Red[i][j] = (float)B->palette[this_bit*4+2]/255;
      }
    }
    return I;

  }
  else
    printf("This number of bits per pixel not supported \n");

  return NULL;
}

