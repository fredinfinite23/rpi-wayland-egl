/*
Copyright (c) 2012, Broadcom Europe Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// A tiny app demonstrating cross-process buffer passing capability/sanity

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "bcm_host.h"
#include "crc.h"

#define PATH "./"

#define IMAGE_SIZE 128
#define CHECKSUM_LEN 64 * 64

int main ()
{
   int result, pip[2];
   pid_t pid;
   FILE *stream;
   VC_RECT_T rect = { 0, 0, IMAGE_SIZE, IMAGE_SIZE };

   // Create a pipe to communication our buffer handle to child process
   printf( "buffer_transmitter : creating the pipe\n" );
   result = pipe(pip);	
   if ( result == -1 )
   {
      fprintf( stderr, "Failed pipe!\n" );
      exit( 1 );
   }

   // fork our child process
   printf( "buffer_transmitter : forking the receiver process\n" );
   if ( ( pid = fork() ) == -1 ) {
      fprintf( stderr, "Failed fork\n" );
      exit( 1 );
   }

   if ( pid != 0 ) {
      // -----------------------------------
      // parent process : buffer transmitter
      // -----------------------------------
      DISPMANX_RESOURCE_HANDLE_T resource;
      DISPMANX_DISPLAY_HANDLE_T dispman_display;
      close( pip[0] );
      uint32_t *buffer, *ptr, vc_image_handle;
      int status;

      bcm_host_init();

      // Create the buffer to pass-over
      printf( "buffer_transmitter : creating dispman buffer...\n" );
      buffer = malloc( sizeof( uint32_t ) * IMAGE_SIZE * IMAGE_SIZE );
      dispman_display = vc_dispmanx_display_open( 0 );
      resource = vc_dispmanx_resource_create( VC_IMAGE_RGBA32, IMAGE_SIZE, IMAGE_SIZE, &vc_image_handle);

      // write random values into the buffer
      printf( "buffer_transmitter : filling up the buffer with random values...\n" );
      srand( time( NULL ) );
      for ( ptr = buffer; ptr < buffer + (IMAGE_SIZE * IMAGE_SIZE); ptr++ )
         *ptr = ( uint32_t )( ( rand() % UINT_MAX ) + 1 );

      if ( vc_dispmanx_resource_write_data( resource, VC_IMAGE_RGBA32, IMAGE_SIZE, buffer, &rect) != 0 ) {
         fprintf( stderr, "Failed to write to VC buffer\n" );
         exit( 1 );
      }
      free( buffer );

      // compute buffer checksum
      crcInit();
      printf( "                                       ------------\n");
      printf( "buffer_transmitter : buffer checksum = 0x%lu\n", 
              crcFast( ( unsigned char* ) buffer, CHECKSUM_LEN * sizeof( uint32_t ) ) ); 
      printf( "                                       ------------\n");

      // pass buffer handle over to forked process
      printf( "buffer_transmitter : passing buffer handle to receiver process\n\n\n" );
      stream = fdopen( pip[1], "w" );
      fprintf( stream, "%u\n", resource );
      fclose( stream );
     
      // wait for child to terminate
      ( void ) waitpid( pid, &status, 0 );

   } else {
      // -------------------------------
      // child process : buffer receiver
      // -------------------------------
      DISPMANX_RESOURCE_HANDLE_T resource;
      close(pip[1]);
      uint32_t *buffer;

      bcm_host_init();

      // wait for the piped handle
      stream = fdopen( pip[0], "r" );
      fscanf( stream, "%u", &resource );
      fclose( stream );

      // retrieve dispman buffer via passed-in handle
      printf( "buffer_receiver ---> retrieving buffer via passed-in handle\n" );
      buffer = malloc( sizeof( uint32_t ) * IMAGE_SIZE * IMAGE_SIZE );
      if ( vc_dispmanx_resource_read_data( resource, &rect, buffer, IMAGE_SIZE ) ) {
         fprintf( stderr, "Failed to read from VC buffer" );
         exit( 1 );
      }

      // compute buffer checksum
      crcInit();
      printf( "                                       ------------\n");
      printf( "buffer_receiver ---> buffer checksum = 0x%lu\n", 
              crcFast( ( unsigned char* ) buffer, CHECKSUM_LEN * sizeof( uint32_t ) ) ); 
      printf( "                                       ------------\n");
      free( buffer );
   }
   return 0;
}

