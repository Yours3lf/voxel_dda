#include "framework.h"
#include "tbb/tbb.h"

using namespace prototyper;

framework frm;

void write_backbuffer( const vec2& pos, const vec4& color );
#define uniform

namespace shader_code
{
//////////////////////////////////////////////////
// Shader code from here
//////////////////////////////////////////////////

struct voxel
{
  unsigned int color; //RGBA8 alpha==0 means empty
};

vec4 get_voxel_color( voxel v )
{
  vec4 r;
  r.x = (v.color >> 24) & 0xff;
  r.y = (v.color >> 16) & 0xff;
  r.z = (v.color >> 8 ) & 0xff;
  r.w = v.color & 0xff;

  return r / 255.0f;
}

//uniforms
uniform float time; //time in seconds
uniform vec4 mouse; //(xy) mouse pos in pixels, (zw) clicks
uniform vec2 resolution; //screen resolution in pixels
uniform vec2 inv_resolution; // 1/resolution
uniform float aspect; // resoltion.y / resolution.x
uniform vec3 sky_color = vec3( 0.678, 0.874, 1.0 );
uniform voxel voxels[32*32*32];
uniform vec3 world_size; //32^3
uniform vec3 voxel_size; //1^3

void set_voxel( const ivec3& p, voxel v )
{
  assert( p.z >= 0 && p.z < 32 && p.y >= 0 && p.y < 32 && p.x >= 0 && p.x < 32 );
  unsigned address = p.z * 32 * 32 + p.y * 32 + p.x;
  voxels[address] = v;
}

voxel get_voxel( const ivec3& p )
{
  assert( p.z >= 0 && p.z < 32 && p.y >= 0 && p.y < 32 && p.x >= 0 && p.x < 32 );
  unsigned address = p.z * 32 * 32 + p.y * 32 + p.x;
  return voxels[address];
}

#ifndef EPSILON
#define EPSILON 0.001f
#endif

#ifndef FLT_MAX
#define FLT_MAX 3.402823466e+38
#endif

#define INVALID (FLT_MAX)

//supersampling positions
uniform vec3 pos00 = vec3( 0.25, 0.25, 0 );
uniform vec3 pos10 = vec3( 0.75, 0.25, 0 );
uniform vec3 pos01 = vec3( 0.25, 0.75, 0 );
uniform vec3 pos11 = vec3( 0.75, 0.75, 0 );

struct ray
{
  vec3 pos, dir;
};

struct aabb
{
  vec3 min, max;
};

struct intersection
{
	float t;
};

struct camera
{
	vec3 pos, view, up, right;
};

uniform camera cam;

//in a shader this would be a simple prng
float rand()
{
  return frm.get_random_num( 0, 1 );
}

intersection intersect_aabb_ray( const aabb& ab, const ray& r )
{
  vec3 invR;

  // compute intersection of ray with all six bbox planes
#ifdef _DEBUG
  //in debug mode, pay attention to asserts
  for(int c = 0; c < 3; ++c )
  {
    if( mm::impl::is_eq(r.dir[c], 0) )
    {
      invR[c] = FLT_MAX;
    }
    else
    {
      invR[c] = 1.0f / r.dir[c];
    }
  }
#else
  //in release mode we dgaf about div by zero
  invR = 1.0f / r.dir;
#endif

	vec3 tbot = invR * (ab.min - r.pos);
	vec3 ttop = invR * (ab.max - r.pos);

	// re-order intersections to find smallest and largest on each axis
	vec3 tmin = min(ttop, tbot);
	vec3 tmax = max(ttop, tbot);

	// find the largest tmin and the smallest tmax
	float largest_tmin = max(max(tmin.x, tmin.y), max(tmin.x, tmin.z));
	float smallest_tmax = min(min(tmax.x, tmax.y), min(tmax.x, tmax.z));

	intersection i;
  i.t = smallest_tmax > largest_tmin ? (largest_tmin >= 0 ? largest_tmin : smallest_tmax) : INVALID;

	return i;
}

vec3 trace( const ray& rr )
{
  /**/
  ray r = rr;
  //DDA Voxel Raycasting

  //place ray's origin into the volume if possible
  vec3 XYZ = floor(r.pos);

  if( XYZ.x < 0 || XYZ.x >= 32 ||
      XYZ.y < 0 || XYZ.y >= 32 ||
      XYZ.z < 0 || XYZ.z >= 32 )
  {
    //if pos is outside the volume, find the intersection

    aabb voxel_aabb;
    voxel_aabb.min = vec3(0);
    voxel_aabb.max = world_size;

    intersection i = intersect_aabb_ray(voxel_aabb, r);

    if( i.t != INVALID )
    {
      r.pos = r.pos + r.dir * (i.t + EPSILON);
      XYZ = floor(r.pos); //recalculate map pos
      XYZ = clamp( XYZ, 0, 31 ); //make sure it's a valid position
    }
    else
    {
      //no intersection
      return -2;
    }
  }

  //DDA algorithm (by Andrew Woo)
  vec3 stepXYZ;
  vec3 deltaXYZ, nextXYZ;
  
  for( int c = 0; c < 3; ++c )
  {
    stepXYZ[c] = r.dir[c] < 0 ? -1 : 1;
  }

#ifdef _DEBUG
  //in debug mode, pay attention to asserts
  for(int c = 0; c < 3; ++c )
  {
    if( mm::impl::is_eq(r.dir[c], 0) )
    {
      deltaXYZ[c] = FLT_MAX;
    }
    else
    {
      deltaXYZ[c] = stepXYZ[c] / r.dir[c];
    }
  }
#else
  //in release mode we dgaf about div by zero
  deltaXYZ = stepXYZ / r.dir;
#endif
  nextXYZ = -stepXYZ * (r.pos - XYZ) * deltaXYZ + max(stepXYZ, 0) * deltaXYZ;

  voxel current;
  vec3 mask;

  while(XYZ.x < 32 && 
        XYZ.y < 32 && 
        XYZ.z < 32 &&
        XYZ.x >= 0 &&
        XYZ.y >= 0 &&
        XYZ.z >= 0 )
  {
    current = get_voxel( ivec3( XYZ.x, XYZ.y, XYZ.z ) );
    if( get_voxel_color( current ).w > 0 )
    {
      return XYZ; //voxel found!
    }

    /*
    vec3 mask0 = step(nextXYZ.xyz, nextXYZ.yzx);
    vec3 mask1 = step(nextXYZ.xyz, nextXYZ.zxy);
    mask = mask0 * mask1; //which coordinate has the smallest value?
    */

    int c = 0;
	  for (int d = 1; d < 3; ++d)
	  {
		  if (nextXYZ[c] > nextXYZ[d])
			  c = d;
	  }

    mask = vec3(0);
	  mask[c] = 1;

    //update next
    nextXYZ += mask * deltaXYZ;
    //move into that direction
    XYZ += mask * stepXYZ;
  }

  return -1; //couldn't find a non-empty voxel
  /**/
}

vec4 calculate_pixel( const vec3& pix_pos )
{
  //2x2 near plane, 90 degrees vertical fov
  vec3 plane_pos = pix_pos * vec3( inv_resolution, 1 ) * 2 - vec3( 1, 1, 0 );
  plane_pos.y *= aspect;

  ray r;
  r.pos = cam.pos + cam.view + cam.right * plane_pos.x + cam.up * plane_pos.y;
  r.dir = normalize(r.pos - cam.pos);

  vec3 pos = trace( r );

  if( pos.x > -1 ) //if found voxel
  {
    voxel current = get_voxel( ivec3( pos.x, pos.y, pos.z ) ); 
    vec4 color = get_voxel_color( current );

    float sum = 0;

    /**/
    aabb current_aabb;
    current_aabb.min = pos;
    current_aabb.max = pos + vec3(1);

    intersection i = intersect_aabb_ray( current_aabb, r );

    //place ray origin slightly above the surface
    vec3 int_point = r.pos + r.dir * ( i.t - 0.001f );

    //calculate AO (doesn't really look correct, but at least shows structural details, so don't care)
    int samples = 8;
    for( int c = 0; c < samples; ++c )
    {
      vec3 dir = normalize( vec3( rand(), rand(), rand() ) * 2 - 1 );
      float coeff = -sign( dot( dir, r.dir ) ); //sample along a hemisphere (would need surface normal, but this should be ok too)
      
      ray box_ray;
      box_ray.pos = int_point;
      box_ray.dir = dir * (coeff != 0 ? coeff : 1); //sign can be 0 
           
      vec3 pos = trace( box_ray );

      sum += pos.x < 0;
    }

    return sum / samples;
    /**/

    return 1;
  }
  else
  {
    //debugging colors
    if( pos.x > -2 )
    {
      return 0; //no hit
    }
    else
    {
      return 0.5;
    }
  }
}

void main( const vec2& gl_FragCoord )
{
  //shader rand() would need this
  //seed += aspect * gl_FragCoord.x + gl_FragCoord.y * inv_resolution.y + time;
  vec2 uv = gl_FragCoord.xy * inv_resolution.xy;

  /**/
  //supersampling
  vec4 color = ( calculate_pixel( vec3( gl_FragCoord.xy, 0 ) + pos00 ) +
	calculate_pixel( vec3( gl_FragCoord.xy, 0 ) + pos01 ) +
	calculate_pixel( vec3( gl_FragCoord.xy, 0 ) + pos10 ) +
	calculate_pixel( vec3( gl_FragCoord.xy, 0 ) + pos11 ) ) * 0.25;
  /**/

  //vec4 color = calculate_pixel( vec3( gl_FragCoord.xy, 0 ) );

  write_backbuffer( gl_FragCoord, color );
}

//////////////////////////////////////////////////
}

//////////////////////////////////////////////////
// Code that runs the raytracer
//////////////////////////////////////////////////

uvec2 screen( 0 );
bool fullscreen = false;
bool silent = false;
string title = "Voxel rendering stuff";
vec4* pixels = 0;

//thread granularity stuff
struct startend
{
  int startx, endx, starty, endy;
};

//this basically divides the screen into 256 small parts
const unsigned threadw = 16;
const unsigned threadh = 16;
startend thread_startends[threadw * threadh];

void thread_func( startend s )
{
  for( int y = s.starty; y < s.endy; y++ )
  {
    for( int x = s.startx; x < s.endx; x++ )
    {
      shader_code::main( vec2( x, y ) );
    }
  }
}

void write_backbuffer( const vec2& pos, const vec4& color )
{
  assert( pos.x < screen.x && pos.y < screen.y && pos.x >= 0 && screen.y >= 0 );

  pixels[int(pos.y * screen.x + pos.x)] = color;
}

vec3 rotate_2d( const vec3& pp, float angle )
{
  vec3 p = pp;
	p.x = p.x * cos( angle ) - p.y * sin( angle );
	p.y = p.y * cos( angle ) + p.x * sin( angle );
	return p;
}

void calculate_ssaa_pos()
{
	float angle = atan( 0.5 );
	float stretch = sqrt( 5.0 ) * 0.5;

	shader_code::pos00 = rotate_2d( shader_code::pos00, angle );
	shader_code::pos01 = rotate_2d( shader_code::pos01, angle );
	shader_code::pos10 = rotate_2d( shader_code::pos10, angle );
	shader_code::pos11 = rotate_2d( shader_code::pos11, angle );

	shader_code::pos00 = ( shader_code::pos00 - vec3( 0.5, 0.5, 0 ) ) * stretch + vec3( 0.5, 0.5, 0 );
	shader_code::pos01 = ( shader_code::pos01 - vec3( 0.5, 0.5, 0 ) ) * stretch + vec3( 0.5, 0.5, 0 );
	shader_code::pos10 = ( shader_code::pos10 - vec3( 0.5, 0.5, 0 ) ) * stretch + vec3( 0.5, 0.5, 0 );
	shader_code::pos11 = ( shader_code::pos11 - vec3( 0.5, 0.5, 0 ) ) * stretch + vec3( 0.5, 0.5, 0 );
}

struct color
{
  unsigned char r, g, b, a;
};

shader_code::camera lookat( const vec3& eye, const vec3& lookat, const vec3& up )
{
	shader_code::camera c;
	c.view = normalize( lookat - eye );
	c.up = normalize( up );
	c.pos = eye;
	c.right = normalize( cross( c.view, c.up ) );
	c.up = normalize( cross( c.right, c.view ) );
	return c;
}

int main( int argc, char** argv )
{
  shape::set_up_intersection();

  map<string, string> args;

  for( int c = 1; c < argc; ++c )
  {
    args[argv[c]] = c + 1 < argc ? argv[c + 1] : "";
    ++c;
  }

  cout << "Arguments: " << endl;
  for_each( args.begin(), args.end(), []( pair<string, string> p )
  {
    cout << p.first << " " << p.second << endl;
  } );

  /*
   * Process program arguments
   */

  stringstream ss;
  ss.str( args["--screenx"] );
  ss >> screen.x;
  ss.clear();
  ss.str( args["--screeny"] );
  ss >> screen.y;
  ss.clear();

  if( screen.x == 0 )
  {
    screen.x = 1280;
  }

  if( screen.y == 0 )
  {
    screen.y = 720;
  }

  try
  {
    args.at( "--fullscreen" );
    fullscreen = true;
  }
  catch( ... ) {}

  try
  {
    args.at( "--help" );
    cout << title << ", written by Marton Tamas." << endl <<
         "Usage: --silent      //don't display FPS info in the terminal" << endl <<
         "       --screenx num //set screen width (default:1280)" << endl <<
         "       --screeny num //set screen height (default:720)" << endl <<
         "       --fullscreen  //set fullscreen, windowed by default" << endl <<
         "       --help        //display this information" << endl;
    return 0;
  }
  catch( ... ) {}

  try
  {
    args.at( "--silent" );
    silent = true;
  }
  catch( ... ) {}

  /*
   * Initialize the OpenGL context
   */

  frm.init( screen, title, fullscreen );

  //set opengl settings
  glEnable( GL_DEPTH_TEST );
  glDepthFunc( GL_LEQUAL );
  glFrontFace( GL_CCW );
  glEnable( GL_CULL_FACE );
  glClearColor( 0.0f, 0.0f, 0.0f, 0.0f );
  glClearDepth( 1.0f );

  frm.get_opengl_error();

  glViewport( 0, 0, screen.x, screen.y );

  /*
   * Set up the scene
   */

   shader_code::resolution = vec2( screen.x, screen.y );
   shader_code::aspect = shader_code::resolution.y / shader_code::resolution.x;
   shader_code::world_size = vec3( 32 );
   shader_code::voxel_size = vec3( 1 );

   pixels = new vec4[screen.x * screen.y];

  //set up the camera
	shader_code::cam = lookat( vec3( 10, 5, 24 ), vec3( 16, 16, 16 ), vec3( 0, 1, 0 ) );

  //calculate thread start/ends
  for( unsigned int x = 0; x < threadw; ++x )
  {
    for( unsigned int y = 0; y < threadh; ++y )
    {
      startend s;
      s.startx = ( screen.x / ( float )threadw ) * x;
      s.endx = ( screen.x / ( float )threadw ) * ( x + 1 );
      s.starty = ( screen.y / ( float )threadh ) * y;
      s.endy = ( screen.y / ( float )threadh ) * ( y + 1 );
      thread_startends[x * threadh + y] = s;
    }
  }

  //initialize thread building blocks
  tbb::task_scheduler_init();

  calculate_ssaa_pos();

  shader_code::inv_resolution = 1.0f / shader_code::resolution;

  int size = 32;

  for( int z = 0; z < size; ++z )
    for( int y = 0; y < size; ++y )
      for( int x = 0; x < size; ++x )
      {
        unsigned color = 0xffffff << 8;
        
        /**/
        //sphere fill
        aabb current = aabb(vec3(x, y, z) + 0.5, 0.5);
        sphere s = sphere( vec3( 16, 16, 16 ), 10 );

        bool i = current.is_intersecting(&s);
        color |= (0xff * i);
        /**/

        //random fill
        //color |= (0xff * (frm.get_random_num( 0, 1 ) > 0.5f));
        
        shader_code::voxel v;
        v.color = color;
        shader_code::set_voxel(ivec3(x, y, z), v);
      }

  /*
   * Handle events
   */

  auto event_handler = [&]( const sf::Event & ev )
  {
    switch( ev.type )
    {
      case sf::Event::MouseMoved:
        {
          shader_code::mouse.x = ev.mouseMove.x;
          shader_code::mouse.y = screen.y - ev.mouseMove.y;
        }
      default:
        break;
    }
  };

  /*
   * Render
   */

  sf::Clock timer;
  timer.restart();

  sf::Clock movement_timer;
  movement_timer.restart();

  float move_amount = 10;
  float orig_move_amount = move_amount;

  vec3 movement_speed = vec3(0);

  cout << "Init finished, rendering starts..." << endl;

  frm.display( [&]
  {
    frm.handle_events( event_handler );

    float seconds = movement_timer.getElapsedTime().asMilliseconds() / 1000.0f;

    if( sf::Keyboard::isKeyPressed( sf::Keyboard::LShift ) || sf::Keyboard::isKeyPressed( sf::Keyboard::RShift ) )
    {
      move_amount = orig_move_amount * 3.0f;
    }
    else
    {
      move_amount = orig_move_amount;
    }

    if( seconds > 0.01667 )
    {
      //move camera
      if( sf::Keyboard::isKeyPressed( sf::Keyboard::A ) )
      {
        movement_speed.x -= move_amount;
      }

      if( sf::Keyboard::isKeyPressed( sf::Keyboard::D ) )
      {
        movement_speed.x += move_amount;
      }

      if( sf::Keyboard::isKeyPressed( sf::Keyboard::W ) )
      {
        movement_speed.y += move_amount;
      }

      if( sf::Keyboard::isKeyPressed( sf::Keyboard::S ) )
      {
        movement_speed.y -= move_amount;
      }

      if( sf::Keyboard::isKeyPressed( sf::Keyboard::Q ) )
      {
        movement_speed.z -= move_amount;
      }

      if( sf::Keyboard::isKeyPressed( sf::Keyboard::E ) )
      {
        movement_speed.z += move_amount;
      }

      {
        shader_code::cam.pos += vec3(0,0,-1) * movement_speed.y * seconds;
        shader_code::cam.pos += vec3(-1,0,0) * movement_speed.x * seconds;
        shader_code::cam.pos += vec3(0,1,0) * movement_speed.z * seconds;

        //set up the camera
	      shader_code::cam = lookat( shader_code::cam.pos, vec3( 16, 16, 16 ), vec3( 0, 1, 0 ) );
      }

      movement_speed *= 0.5;

      movement_timer.restart();
    }

    shader_code::time = timer.getElapsedTime().asMilliseconds() * 0.001f;

    //raytrace for each pixel in parallel
    /**/
    tbb::parallel_for( tbb::blocked_range<size_t>( 0, threadw * threadh ),
    [ = ]( const tbb::blocked_range<size_t>& r )
    {
      for( size_t i = r.begin(); i != r.end(); ++i )
        thread_func( thread_startends[i] );
    });
    /**/

    /**
    //single threaded code for debugging
    for( int y = 0; y < screen.y; ++y )
      for( int x = 0; x < screen.x; ++x )
      {
        shader_code::main( vec2( x, y ) );
      }
    /**/

    glDrawPixels( screen.x, screen.y, GL_RGBA, GL_FLOAT, pixels );

    //cout << "frame" << endl;

    frm.get_opengl_error();
  }, silent );

  return 0;
}
