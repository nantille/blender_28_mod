/*
 * Copyright 2011-2013 Blender Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "render/attribute.h"
#include "render/camera.h"
#include "render/curves.h"
#include "render/mesh.h"
#include "render/object.h"
#include "render/scene.h"

#include "blender/blender_sync.h"
#include "blender/blender_util.h"

#include "util/util_foreach.h"
#include "util/util_hash.h"
#include "util/util_logging.h"

CCL_NAMESPACE_BEGIN

ParticleCurveData::ParticleCurveData()
{
}

ParticleCurveData::~ParticleCurveData()
{
}

static void interp_weights(float t, float data[4])
{
  /* Cardinal curve interpolation */
  float t2 = t * t;
  float t3 = t2 * t;
  float fc = 0.71f;

  data[0] = -fc * t3 + 2.0f * fc * t2 - fc * t;
  data[1] = (2.0f - fc) * t3 + (fc - 3.0f) * t2 + 1.0f;
  data[2] = (fc - 2.0f) * t3 + (3.0f - 2.0f * fc) * t2 + fc * t;
  data[3] = fc * t3 - fc * t2;
}

static void curveinterp_v3_v3v3v3v3(
    float3 *p, float3 *v1, float3 *v2, float3 *v3, float3 *v4, const float w[4])
{
  p->x = v1->x * w[0] + v2->x * w[1] + v3->x * w[2] + v4->x * w[3];
  p->y = v1->y * w[0] + v2->y * w[1] + v3->y * w[2] + v4->y * w[3];
  p->z = v1->z * w[0] + v2->z * w[1] + v3->z * w[2] + v4->z * w[3];
}

static float shaperadius(float shape, float root, float tip, float time)
{
  assert(time >= 0.0f);
  assert(time <= 1.0f);
  float radius = 1.0f - time;

  if (shape != 0.0f) {
    if (shape < 0.0f)
      radius = powf(radius, 1.0f + shape);
    else
      radius = powf(radius, 1.0f / (1.0f - shape));
  }
  return (radius * (root - tip)) + tip;
}

/* curve functions */

static void InterpolateKeySegments(
    int seg, int segno, int key, int curve, float3 *keyloc, float *time, ParticleCurveData *CData)
{
  float3 ckey_loc1 = CData->curvekey_co[key];
  float3 ckey_loc2 = ckey_loc1;
  float3 ckey_loc3 = CData->curvekey_co[key + 1];
  float3 ckey_loc4 = ckey_loc3;

  if (key > CData->curve_firstkey[curve])
    ckey_loc1 = CData->curvekey_co[key - 1];

  if (key < CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 2)
    ckey_loc4 = CData->curvekey_co[key + 2];

  float time1 = CData->curvekey_time[key] / CData->curve_length[curve];
  float time2 = CData->curvekey_time[key + 1] / CData->curve_length[curve];

  float dfra = (time2 - time1) / (float)segno;

  if (time)
    *time = (dfra * seg) + time1;

  float t[4];

  interp_weights((float)seg / (float)segno, t);

  if (keyloc)
    curveinterp_v3_v3v3v3v3(keyloc, &ckey_loc1, &ckey_loc2, &ckey_loc3, &ckey_loc4, t);
}

// Very similar to BKE_curve_forward_diff_bezier() from source/blender/blenkernel/intern/curve.c
static void forward_diff_bezier(const float3& q0,
                                const float3& q1,
                                const float3& q2,
                                const float3& q3,
                                int resolution,
                                vector<float3> *points)
{
	const float resolution2 = resolution*resolution;
	const float resolution3 = resolution2*resolution;
	const float3 rt0 = q0;
	const float3 rt1 = 3.0f * (q1 - q0) / resolution;
	const float3 rt2 = 3.0f * (q0 - 2.0f * q1 + q2) / resolution2;
	const float3 rt3 = (q3 - q0 + 3.0f * (q1 - q2)) / resolution3;
	float3 a0 = rt0;
	float3 a1 = rt1 + rt2 + rt3;
	float3 a2 = 2 * rt2 + 6 * rt3;
	float3 a3 = 6 * rt3;
  //if (points->capacity() < resolution + 1)
	//points->resize(resolution + 1);
	for (int i = 0; i <= resolution; ++i) 
  {
		(*points)[i] = a0;
		a0 += a1;
		a1 += a2;
		a2 += a3;
	}
}

/* 
This is the transition part. We get data from Blender and prepare it for Cycles.
Here, we handle splines and curve data. Be warned about variable names: 
in C/C++ sources of Cycles, "curve" describes both curve objects and splines (!).
Also, custom data was added to SplinePoint and BezierSplinePoint, namely "key" and "value",
two 4 byte float that can store custom data that will be reflected and available in
HairInfo node as HairInfo.Key and HairInfo.Value.
Some of the features that are supported here:
- per spline materials using material_index attribute
- proper nurbs and bezier interpolations
- resolution == 1 makes a copy of raw data without interp, which is faster
*/
static bool ObtainCurveData(Mesh *mesh,
                            BL::Mesh *b_mesh,
                            BL::Object *b_ob,
                            ParticleCurveData *CData,
                            bool background)
{
  if(!(mesh && b_mesh && b_ob && CData))
      return false;
    
  BL::ID b_ob_data = b_ob->data();
  if(!b_ob_data || !b_ob_data.is_a(&RNA_Curve))
    return false;

	BL::Curve b_curve(b_ob->data());
	const int num_splines = b_curve.splines.length();
	if(num_splines == 0)
		return false;

	bool render_as_hair = false;
  float reveal_amount = 1.0f;
	if(!b_ob_data || b_ob_data.is_a(&RNA_Curve)) 
  {
    // "cycles_curves" is here a property of the Curve data 
		PointerRNA cycles_curves = RNA_pointer_get(&b_ob_data.ptr, "cycles_curves");
		render_as_hair = get_boolean(cycles_curves, "render_as_hair");
    reveal_amount = b_curve.bevel_factor_end();
	}

  if (reveal_amount == 0.0f)
    return false;

	if(!render_as_hair)
    return false;

	CData->psys_firstcurve.push_back_slow(0);
	CData->psys_curvenum.push_back_slow(num_splines);
  CData->render_as_hair.push_back_slow(render_as_hair);

  CData->curve_keynum.reserve(num_splines);
  CData->curve_shader.reserve(num_splines);
  CData->curve_length.reserve(num_splines);
  CData->curve_firstkey.reserve(num_splines);
  
  //vector<float> curves_entire_length;
  //curves_entire_length.reserve(num_splines);
  float all_curves_length = 0.0;
  float max_curve_length = 0.0;
  
  // Computing lengths beforehand so that we can stop adding points if the user chose 
  // to reveal only parts of the splines.
  for(int spline = 0; spline < num_splines; ++spline) 
  {
    BL::Spline b_spline = b_curve.splines[spline];
    const bool is_bezier = (b_spline.type() == BL::Spline::type_BEZIER);
    const int num_points = is_bezier ? b_spline.bezier_points.length() : b_spline.points.length();

		float curve_length = 0.0f;
		float3 prev_co;
    for(int point = 0; point <= num_points-1; ++point) 
    {
      if(is_bezier)
      {
        BL::BezierSplinePoint b_curr_point = b_spline.bezier_points[point];

        const float3 co = get_float3(b_curr_point.co());
        if(point != 0) 
        {
          const float step_length = len(co - prev_co);
          if(step_length == 0.0f)
            continue;
          curve_length += step_length;
        }
        prev_co = co;
      }
      else
      {
        BL::SplinePoint b_curr_point = b_spline.points[point];

        // Forced to duplicate code here, there's no abstract class for BezierSplinePoint and SplinePoint
        const float3 co = get_float3(b_curr_point.co());
        if(point != 0) 
        {
          const float step_length = len(co - prev_co);
          if(step_length == 0.0f)
            continue;
          curve_length += step_length;
        }
        prev_co = co;
      }
    }
    if (max_curve_length < curve_length)
        max_curve_length = curve_length;
    all_curves_length += curve_length;
    //curves_entire_length.push_back(curve_length);
  }
  //double max_length = *max_element(curves_entire_length.begin(), curves_entire_length.end());

  const float reveal_threshold = all_curves_length * reveal_amount;
  float current_curves_length = 0.0f;
	int keyno = 0;
  // Variables refer to splines here as this is really what you have in the UI.
  // In Cycles code, splines are curves and curves are "psys" or particle systems.
	for(int spline = 0; spline < num_splines; ++spline) 
  {
		BL::Spline b_spline = b_curve.splines[spline];
    const bool is_bezier = (b_spline.type() == BL::Spline::type_BEZIER);
		const int num_points = is_bezier ? b_spline.bezier_points.length() : b_spline.points.length();
		int resolution = background ? b_curve.render_resolution_u() : b_spline.resolution_u();
		if(resolution == 0)
			resolution = b_curve.resolution_u();
    if(resolution == 0)
      resolution = 1;

		CData->curve_firstkey.push_back_slow(keyno);
    const int shader = clamp(b_spline.material_index(), 0, mesh->used_shaders.size()-1);
    CData->curve_shader.push_back_slow(shader);

    // If resolution == 1, these reserve() are still valid.
    // Older code used resize() + push_back_slow(), now it's reserve() + push_back_slow()
    // which should be slightly faster.
    const int reserve_size = num_points * resolution;
    CData->curvekey_co.reserve(CData->curvekey_co.capacity() + reserve_size);
    CData->curvekey_radius.reserve(CData->curvekey_radius.capacity() + reserve_size);
    CData->curvekey_time.reserve(CData->curvekey_time.capacity() + reserve_size);
    CData->curvekey_key.reserve(CData->curvekey_key.capacity() + reserve_size);
    CData->curvekey_value.reserve(CData->curvekey_value.capacity() + reserve_size);

		int keynum = 0;
		float curve_length = 0.0f;
		float3 prev_co;

    // When resolution equals 1, no interpolation is performed, we directly copy the given data
    if (resolution == 1)
    {
      for(int point = 0; point <= num_points-1; ++point) 
      {
        if(is_bezier) 
        {
          BL::BezierSplinePoint b_curr_point = b_spline.bezier_points[point];

          const float3 co = get_float3(b_curr_point.co());
          if(point != 0) 
          {
            const float step_length = len(co - prev_co);
            if(step_length == 0.0f)
              continue;
            curve_length += step_length;
            current_curves_length += step_length;
            if(reveal_amount < 1.0f && current_curves_length > reveal_threshold)
              break;
          }
          CData->curvekey_co.push_back_slow(co);
          CData->curvekey_radius.push_back_slow(b_curr_point.radius());
          CData->curvekey_time.push_back_slow(curve_length);
          CData->curvekey_key.push_back_slow(b_curr_point.key());
          CData->curvekey_value.push_back_slow(b_curr_point.value());
          prev_co = co;
          }
          else
          {
            BL::SplinePoint b_curr_point = b_spline.points[point];

            // Forced to duplicate code here, there's no abstract class for BezierSplinePoint and SplinePoint
            const float3 co = get_float3(b_curr_point.co());
            if(point != 0) 
            {
              const float step_length = len(co - prev_co);
              if(step_length == 0.0f)
                continue;
              curve_length += step_length;
              current_curves_length += step_length;
              if(reveal_amount < 1.0f && current_curves_length > reveal_threshold)
                break;
            }
            CData->curvekey_co.push_back_slow(co);
            CData->curvekey_radius.push_back_slow(b_curr_point.radius());
            CData->curvekey_time.push_back_slow(curve_length);
            CData->curvekey_key.push_back_slow(b_curr_point.key());
            CData->curvekey_value.push_back_slow(b_curr_point.value());
            prev_co = co;
          }
        }
        CData->curve_keynum.push_back_slow(num_points);
        CData->curve_length.push_back_slow(curve_length);
        continue;
      }
        
    // Interpolation of given spline points when resolution larger than 1
    for(int point = 0; point < num_points - 1; ++point) 
    {
      vector<float3> points(resolution + 1);
      vector<float> radii(resolution + 1);
      vector<float> custom_data_keys(resolution + 1);
      vector<float> custom_data_values(resolution + 1);
      if(is_bezier) 
      {
        BL::BezierSplinePoint b_curr_point = b_spline.bezier_points[point];
        BL::BezierSplinePoint b_next_point = b_spline.bezier_points[point + 1];
        const float3 q0 = get_float3(b_curr_point.co());
        const float3 q1 = get_float3(b_curr_point.handle_right());
        const float3 q2 = get_float3(b_next_point.handle_left());
        const float3 q3 = get_float3(b_next_point.co());
        forward_diff_bezier(q0, q1, q2, q3, resolution, &points);

        // Data could be evaluated with a bezier spline. Currently using linear interpolation.
        for (int i = 0; i <= resolution; ++i)
        {
          radii[i] = b_curr_point.radius() + (b_next_point.radius() - b_curr_point.radius()) / resolution * i;
          custom_data_keys[i] = b_curr_point.key() + (b_next_point.key() - b_curr_point.key()) / resolution * i;
          custom_data_values[i] = b_curr_point.value() + (b_next_point.value() - b_curr_point.value()) / resolution * i;
        }
      }
      else 
      {
        BL::SplinePoint b_curr_point = b_spline.points[point];
        BL::SplinePoint b_next_point = b_spline.points[point + 1];

        for (int i = 0; i <= resolution; ++i) 
        {
          float3 ckey_loc1 = get_float3(b_curr_point.co());
          float3 ckey_loc2 = ckey_loc1;
          float3 ckey_loc3 = get_float3(b_next_point.co());
          float3 ckey_loc4 = ckey_loc3;

          if(point > 0) 
          {
            BL::SplinePoint b_prev_point = b_spline.points[point-1];
            ckey_loc1 = get_float3(b_prev_point.co());
          }

          if(point < num_points - 2)
          {
            BL::SplinePoint b_second_next_point = b_spline.points[point+2];
            ckey_loc4 = get_float3(b_second_next_point.co());
          }

          float weights[4];
          interp_weights((float)i / (float)resolution, weights);

          float3 interp_point;
          curveinterp_v3_v3v3v3v3(&interp_point, &ckey_loc1, &ckey_loc2, &ckey_loc3, &ckey_loc4, weights);
          points[i] = interp_point;

          // Data could be evaluated with a nurbs. Currently using linear interpolation.
          radii[i] = b_curr_point.radius() + (b_next_point.radius() - b_curr_point.radius()) / resolution * i;
          custom_data_keys[i] = b_curr_point.key() + (b_next_point.key() - b_curr_point.key()) / resolution * i;
          custom_data_values[i] = b_curr_point.value() + (b_next_point.value() - b_curr_point.value()) / resolution * i;
        }
      }

      // Store the interpolated values
      for (int interp_i = 0; interp_i <= resolution; ++interp_i) 
      {
        const float3 co = points[interp_i];
        const float radius = radii[interp_i];
        const float custom_data_key = custom_data_keys[interp_i];
        const float custom_data_value = custom_data_values[interp_i];

        if(!(point == 0 && interp_i == 0)) 
        {
          const float step_length = len(co - prev_co);
          if(step_length == 0.0f)
            continue;
          curve_length += step_length;
          current_curves_length += step_length;
          if(reveal_amount < 1.0f && current_curves_length > reveal_threshold)
              break;
        }
        CData->curvekey_co.push_back_slow(co);
        CData->curvekey_radius.push_back_slow(radius);
        CData->curvekey_time.push_back_slow(curve_length);
        CData->curvekey_key.push_back_slow(custom_data_key);
        CData->curvekey_value.push_back_slow(custom_data_value);

        prev_co = co;
        ++keynum;
			}
		}
		keyno += keynum;

		CData->curve_keynum.push_back_slow(keynum);
		CData->curve_length.push_back_slow(curve_length);
	}
	return true;
}
    
static bool ObtainCacheParticleData(
    Mesh *mesh, BL::Mesh *b_mesh, BL::Object *b_ob, ParticleCurveData *CData, bool background)
{
  int curvenum = 0;
  int keyno = 0;

  if (!(mesh && b_mesh && b_ob && CData))
    return false;

  Transform tfm = get_transform(b_ob->matrix_world());
  Transform itfm = transform_quick_inverse(tfm);

  BL::Object::modifiers_iterator b_mod;
  for (b_ob->modifiers.begin(b_mod); b_mod != b_ob->modifiers.end(); ++b_mod) {
    if ((b_mod->type() == b_mod->type_PARTICLE_SYSTEM) &&
        (background ? b_mod->show_render() : b_mod->show_viewport())) {
      BL::ParticleSystemModifier psmd((const PointerRNA)b_mod->ptr);
      BL::ParticleSystem b_psys((const PointerRNA)psmd.particle_system().ptr);
      BL::ParticleSettings b_part((const PointerRNA)b_psys.settings().ptr);

      if ((b_part.render_type() == BL::ParticleSettings::render_type_PATH) &&
          (b_part.type() == BL::ParticleSettings::type_HAIR)) {
        const int shader = clamp(b_part.material() - 1, 0, mesh->used_shaders.size() - 1);
        const int display_step = background ? b_part.render_step() : b_part.display_step();
        const int totparts = b_psys.particles.length();
        int totchild = background ? b_psys.child_particles.length() :
                                    (int)((float)b_psys.child_particles.length() *
                                          (float)b_part.display_percentage() / 100.0f);
        int totcurves = totchild;

        if (b_part.child_type() == 0 || totchild == 0)
          totcurves += totparts;

        if (totcurves == 0)
          continue;

        int ren_step = (1 << display_step) + 1;
        if (b_part.kink() == BL::ParticleSettings::kink_SPIRAL)
          ren_step += b_part.kink_extra_steps();

        CData->psys_firstcurve.push_back_slow(curvenum);
        CData->psys_curvenum.push_back_slow(totcurves);
        CData->psys_shader.push_back_slow(shader);

        float radius = b_part.radius_scale() * 0.5f;

        CData->psys_rootradius.push_back_slow(radius * b_part.root_radius());
        CData->psys_tipradius.push_back_slow(radius * b_part.tip_radius());
        CData->psys_shape.push_back_slow(b_part.shape());
        CData->psys_closetip.push_back_slow(b_part.use_close_tip());

        int pa_no = 0;
        if (!(b_part.child_type() == 0) && totchild != 0)
          pa_no = totparts;

        int num_add = (totparts + totchild - pa_no);
        CData->curve_firstkey.reserve(CData->curve_firstkey.size() + num_add);
        CData->curve_keynum.reserve(CData->curve_keynum.size() + num_add);
        CData->curve_length.reserve(CData->curve_length.size() + num_add);
        CData->curvekey_co.reserve(CData->curvekey_co.size() + num_add * ren_step);
        CData->curvekey_time.reserve(CData->curvekey_time.size() + num_add * ren_step);

        for (; pa_no < totparts + totchild; pa_no++) {
          int keynum = 0;
          CData->curve_firstkey.push_back_slow(keyno);

          float curve_length = 0.0f;
          float3 pcKey;
          for (int step_no = 0; step_no < ren_step; step_no++) {
            float nco[3];
            b_psys.co_hair(*b_ob, pa_no, step_no, nco);
            float3 cKey = make_float3(nco[0], nco[1], nco[2]);
            cKey = transform_point(&itfm, cKey);
            if (step_no > 0) {
              const float step_length = len(cKey - pcKey);
              curve_length += step_length;
            }
            CData->curvekey_co.push_back_slow(cKey);
            CData->curvekey_time.push_back_slow(curve_length);
            // [Nicolas Antille] Hair from a p-system could be given key-value data through python API
            // But I'm not sure how... TODO later
            CData->curvekey_key.push_back_slow(0.0);
            CData->curvekey_value.push_back_slow(0.0);
            pcKey = cKey;
            keynum++;
          }
          keyno += keynum;

          CData->curve_keynum.push_back_slow(keynum);
          CData->curve_length.push_back_slow(curve_length);
          curvenum++;
        }
      }
    }
  }

  return true;
}

static bool ObtainCacheParticleUV(Mesh *mesh,
                                  BL::Mesh *b_mesh,
                                  BL::Object *b_ob,
                                  ParticleCurveData *CData,
                                  bool background,
                                  int uv_num)
{
  if (!(mesh && b_mesh && b_ob && CData))
    return false;

  CData->curve_uv.clear();

  BL::Object::modifiers_iterator b_mod;
  for (b_ob->modifiers.begin(b_mod); b_mod != b_ob->modifiers.end(); ++b_mod) {
    if ((b_mod->type() == b_mod->type_PARTICLE_SYSTEM) &&
        (background ? b_mod->show_render() : b_mod->show_viewport())) {
      BL::ParticleSystemModifier psmd((const PointerRNA)b_mod->ptr);
      BL::ParticleSystem b_psys((const PointerRNA)psmd.particle_system().ptr);
      BL::ParticleSettings b_part((const PointerRNA)b_psys.settings().ptr);

      if ((b_part.render_type() == BL::ParticleSettings::render_type_PATH) &&
          (b_part.type() == BL::ParticleSettings::type_HAIR)) {
        int totparts = b_psys.particles.length();
        int totchild = background ? b_psys.child_particles.length() :
                                    (int)((float)b_psys.child_particles.length() *
                                          (float)b_part.display_percentage() / 100.0f);
        int totcurves = totchild;

        if (b_part.child_type() == 0 || totchild == 0)
          totcurves += totparts;

        if (totcurves == 0)
          continue;

        int pa_no = 0;
        if (!(b_part.child_type() == 0) && totchild != 0)
          pa_no = totparts;

        int num_add = (totparts + totchild - pa_no);
        CData->curve_uv.reserve(CData->curve_uv.size() + num_add);

        BL::ParticleSystem::particles_iterator b_pa;
        b_psys.particles.begin(b_pa);
        for (; pa_no < totparts + totchild; pa_no++) {
          /* Add UVs */
          BL::Mesh::uv_layers_iterator l;
          b_mesh->uv_layers.begin(l);

          float2 uv = make_float2(0.0f, 0.0f);
          if (b_mesh->uv_layers.length())
            b_psys.uv_on_emitter(psmd, *b_pa, pa_no, uv_num, &uv.x);
          CData->curve_uv.push_back_slow(uv);

          if (pa_no < totparts && b_pa != b_psys.particles.end())
            ++b_pa;
        }
      }
    }
  }

  return true;
}

static bool ObtainCacheParticleVcol(Mesh *mesh,
                                    BL::Mesh *b_mesh,
                                    BL::Object *b_ob,
                                    ParticleCurveData *CData,
                                    bool background,
                                    int vcol_num)
{
  if (!(mesh && b_mesh && b_ob && CData))
    return false;

  CData->curve_vcol.clear();

  BL::Object::modifiers_iterator b_mod;
  for (b_ob->modifiers.begin(b_mod); b_mod != b_ob->modifiers.end(); ++b_mod) {
    if ((b_mod->type() == b_mod->type_PARTICLE_SYSTEM) &&
        (background ? b_mod->show_render() : b_mod->show_viewport())) {
      BL::ParticleSystemModifier psmd((const PointerRNA)b_mod->ptr);
      BL::ParticleSystem b_psys((const PointerRNA)psmd.particle_system().ptr);
      BL::ParticleSettings b_part((const PointerRNA)b_psys.settings().ptr);

      if ((b_part.render_type() == BL::ParticleSettings::render_type_PATH) &&
          (b_part.type() == BL::ParticleSettings::type_HAIR)) {
        int totparts = b_psys.particles.length();
        int totchild = background ? b_psys.child_particles.length() :
                                    (int)((float)b_psys.child_particles.length() *
                                          (float)b_part.display_percentage() / 100.0f);
        int totcurves = totchild;

        if (b_part.child_type() == 0 || totchild == 0)
          totcurves += totparts;

        if (totcurves == 0)
          continue;

        int pa_no = 0;
        if (!(b_part.child_type() == 0) && totchild != 0)
          pa_no = totparts;

        int num_add = (totparts + totchild - pa_no);
        CData->curve_vcol.reserve(CData->curve_vcol.size() + num_add);

        BL::ParticleSystem::particles_iterator b_pa;
        b_psys.particles.begin(b_pa);
        for (; pa_no < totparts + totchild; pa_no++) {
          /* Add vertex colors */
          BL::Mesh::vertex_colors_iterator l;
          b_mesh->vertex_colors.begin(l);

          float3 vcol = make_float3(0.0f, 0.0f, 0.0f);
          if (b_mesh->vertex_colors.length())
            b_psys.mcol_on_emitter(psmd, *b_pa, pa_no, vcol_num, &vcol.x);
          CData->curve_vcol.push_back_slow(vcol);

          if (pa_no < totparts && b_pa != b_psys.particles.end())
            ++b_pa;
        }
      }
    }
  }

  return true;
}

static void ExportCurveTrianglePlanes(Mesh *mesh,
                                      ParticleCurveData *CData,
                                      float3 RotCam,
                                      bool is_ortho)
{
  int vertexno = mesh->verts.size();
  int vertexindex = vertexno;
  int numverts = 0, numtris = 0;

  /* compute and reserve size of arrays */
  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) {
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) {
      numverts += 2 + (CData->curve_keynum[curve] - 1) * 2;
      numtris += (CData->curve_keynum[curve] - 1) * 2;
    }
  }

  mesh->reserve_mesh(mesh->verts.size() + numverts, mesh->num_triangles() + numtris);

  /* actually export */
  bool render_as_hair = false;
  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) {
    if (CData->render_as_hair.size() > sys)
      render_as_hair = CData->render_as_hair[sys];
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) {
      float3 xbasis;
      float3 v1;
      float time = 0.0f;
      float3 ickey_loc = CData->curvekey_co[CData->curve_firstkey[curve]];
      float radius;
      if (render_as_hair) {
        radius = CData->curvekey_radius[CData->curve_firstkey[curve]];
      }
      else {
        radius = shaperadius(CData->psys_shape[sys], CData->psys_rootradius[sys], CData->psys_tipradius[sys], 0.0f);
      }
      v1 = CData->curvekey_co[CData->curve_firstkey[curve] + 1] -
           CData->curvekey_co[CData->curve_firstkey[curve]];
      if (is_ortho)
        xbasis = normalize(cross(RotCam, v1));
      else
        xbasis = normalize(cross(RotCam - ickey_loc, v1));
      float3 ickey_loc_shfl = ickey_loc - radius * xbasis;
      float3 ickey_loc_shfr = ickey_loc + radius * xbasis;
      mesh->add_vertex(ickey_loc_shfl);
      mesh->add_vertex(ickey_loc_shfr);
      vertexindex += 2;
        
      int shader;
      if (render_as_hair)
        // "curve" variable is referring to a spline here
        shader = CData->curve_shader[curve];
      else
        shader = CData->psys_shader[sys];

      for (int curvekey = CData->curve_firstkey[curve] + 1;
           curvekey < CData->curve_firstkey[curve] + CData->curve_keynum[curve];
           curvekey++) {
        ickey_loc = CData->curvekey_co[curvekey];

        if (curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1)
          v1 = CData->curvekey_co[curvekey] -
               CData->curvekey_co[max(curvekey - 1, CData->curve_firstkey[curve])];
        else
          v1 = CData->curvekey_co[curvekey + 1] - CData->curvekey_co[curvekey - 1];

        time = CData->curvekey_time[curvekey] / CData->curve_length[curve];
        if (render_as_hair) {
          radius = CData->curvekey_radius[curvekey];
        }
        else{
          radius = shaperadius(
              CData->psys_shape[sys], CData->psys_rootradius[sys], CData->psys_tipradius[sys], time);

          if (curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1)
            radius = shaperadius(CData->psys_shape[sys],
                                 CData->psys_rootradius[sys],
                                 CData->psys_tipradius[sys],
                                 0.95f);

          if (CData->psys_closetip[sys] &&
              (curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1))
            radius = shaperadius(CData->psys_shape[sys], CData->psys_rootradius[sys], 0.0f, 0.95f);
        }

        if (is_ortho)
          xbasis = normalize(cross(RotCam, v1));
        else
          xbasis = normalize(cross(RotCam - ickey_loc, v1));
        float3 ickey_loc_shfl = ickey_loc - radius * xbasis;
        float3 ickey_loc_shfr = ickey_loc + radius * xbasis;
        mesh->add_vertex(ickey_loc_shfl);
        mesh->add_vertex(ickey_loc_shfr);
        mesh->add_triangle(
            vertexindex - 2, vertexindex, vertexindex - 1, shader, true);
        mesh->add_triangle(
            vertexindex + 1, vertexindex - 1, vertexindex, shader, true);
        vertexindex += 2;
      }
    }
  }

  mesh->resize_mesh(mesh->verts.size(), mesh->num_triangles());
  mesh->attributes.remove(ATTR_STD_VERTEX_NORMAL);
  mesh->attributes.remove(ATTR_STD_FACE_NORMAL);
  mesh->add_face_normals();
  mesh->add_vertex_normals();
  mesh->attributes.remove(ATTR_STD_FACE_NORMAL);

  /* texture coords still needed */
}

static void ExportCurveTriangleGeometry(Mesh *mesh, ParticleCurveData *CData, int resolution)
{
  int vertexno = mesh->verts.size();
  int vertexindex = vertexno;
  int numverts = 0, numtris = 0;

  /* compute and reserve size of arrays */
  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) {
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) {
      numverts += (CData->curve_keynum[curve] - 1) * resolution + resolution;
      numtris += (CData->curve_keynum[curve] - 1) * 2 * resolution;
    }
  }

  mesh->reserve_mesh(mesh->verts.size() + numverts, mesh->num_triangles() + numtris);

  /* actually export */
  bool render_as_hair = false;
  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) {
    if (CData->render_as_hair.size() > sys)
      render_as_hair = CData->render_as_hair[sys];
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) {
      float3 firstxbasis = cross(make_float3(1.0f, 0.0f, 0.0f),
                                 CData->curvekey_co[CData->curve_firstkey[curve] + 1] -
                                     CData->curvekey_co[CData->curve_firstkey[curve]]);
      if (!is_zero(firstxbasis))
        firstxbasis = normalize(firstxbasis);
      else
        firstxbasis = normalize(cross(make_float3(0.0f, 1.0f, 0.0f),
                                      CData->curvekey_co[CData->curve_firstkey[curve] + 1] -
                                          CData->curvekey_co[CData->curve_firstkey[curve]]));

      for (int curvekey = CData->curve_firstkey[curve];
           curvekey < CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1;
           curvekey++) {
        float3 xbasis = firstxbasis;
        float3 v1;
        float3 v2;

        if (curvekey == CData->curve_firstkey[curve]) {
          v1 = CData->curvekey_co[min(
                   curvekey + 2, CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1)] -
               CData->curvekey_co[curvekey + 1];
          v2 = CData->curvekey_co[curvekey + 1] - CData->curvekey_co[curvekey];
        }
        else if (curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1) {
          v1 = CData->curvekey_co[curvekey] - CData->curvekey_co[curvekey - 1];
          v2 = CData->curvekey_co[curvekey - 1] -
               CData->curvekey_co[max(curvekey - 2, CData->curve_firstkey[curve])];
        }
        else {
          v1 = CData->curvekey_co[curvekey + 1] - CData->curvekey_co[curvekey];
          v2 = CData->curvekey_co[curvekey] - CData->curvekey_co[curvekey - 1];
        }

        xbasis = cross(v1, v2);

        if (len_squared(xbasis) >= 0.05f * len_squared(v1) * len_squared(v2)) {
          firstxbasis = normalize(xbasis);
          break;
        }
      }
        
      int shader;
      if (render_as_hair)
        // "curve" variable is actually referring to a spline here
        shader = CData->curve_shader[curve];
      else
        shader = CData->psys_shader[sys];

      for (int curvekey = CData->curve_firstkey[curve];
           curvekey < CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1;
           curvekey++) {
        int subv = 1;
        float3 xbasis;
        float3 ybasis;
        float3 v1;
        float3 v2;

        if (curvekey == CData->curve_firstkey[curve]) {
          subv = 0;
          v1 = CData->curvekey_co[min(
                   curvekey + 2, CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1)] -
               CData->curvekey_co[curvekey + 1];
          v2 = CData->curvekey_co[curvekey + 1] - CData->curvekey_co[curvekey];
        }
        else if (curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1) {
          v1 = CData->curvekey_co[curvekey] - CData->curvekey_co[curvekey - 1];
          v2 = CData->curvekey_co[curvekey - 1] -
               CData->curvekey_co[max(curvekey - 2, CData->curve_firstkey[curve])];
        }
        else {
          v1 = CData->curvekey_co[curvekey + 1] - CData->curvekey_co[curvekey];
          v2 = CData->curvekey_co[curvekey] - CData->curvekey_co[curvekey - 1];
        }

        xbasis = cross(v1, v2);

        if (len_squared(xbasis) >= 0.05f * len_squared(v1) * len_squared(v2)) {
          xbasis = normalize(xbasis);
          firstxbasis = xbasis;
        }
        else
          xbasis = firstxbasis;

        ybasis = normalize(cross(xbasis, v2));

        for (; subv <= 1; subv++) {
          float3 ickey_loc = make_float3(0.0f, 0.0f, 0.0f);
          float time = 0.0f;
          float radius;
          InterpolateKeySegments(subv, 1, curvekey, curve, &ickey_loc, &time, CData);

          if (render_as_hair) {
            radius = CData->curvekey_radius[curvekey];
          }
          else {
            radius = shaperadius(CData->psys_shape[sys],
                                       CData->psys_rootradius[sys],
                                       CData->psys_tipradius[sys],
                                       time);

            if ((curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 2) &&
                (subv == 1))
              radius = shaperadius(CData->psys_shape[sys],
                                   CData->psys_rootradius[sys],
                                   CData->psys_tipradius[sys],
                                   0.95f);

            if (CData->psys_closetip[sys] && (subv == 1) &&
                (curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 2))
              radius = shaperadius(CData->psys_shape[sys], CData->psys_rootradius[sys], 0.0f, 0.95f);
          }

          float angle = M_2PI_F / (float)resolution;
          for (int section = 0; section < resolution; section++) {
            float3 ickey_loc_shf = ickey_loc + radius * (cosf(angle * section) * xbasis +
                                                         sinf(angle * section) * ybasis);
            mesh->add_vertex(ickey_loc_shf);
          }

          if (subv != 0) {
            for (int section = 0; section < resolution - 1; section++) {
              mesh->add_triangle(vertexindex - resolution + section,
                                 vertexindex + section,
                                 vertexindex - resolution + section + 1,
                                 shader,
                                 true);
              mesh->add_triangle(vertexindex + section + 1,
                                 vertexindex - resolution + section + 1,
                                 vertexindex + section,
                                 shader,
                                 true);
            }
            mesh->add_triangle(vertexindex - 1,
                               vertexindex + resolution - 1,
                               vertexindex - resolution,
                               shader,
                               true);
            mesh->add_triangle(vertexindex,
                               vertexindex - resolution,
                               vertexindex + resolution - 1,
                               shader,
                               true);
          }
          vertexindex += resolution;
        }
      }
    }
  }

  mesh->resize_mesh(mesh->verts.size(), mesh->num_triangles());
  mesh->attributes.remove(ATTR_STD_VERTEX_NORMAL);
  mesh->attributes.remove(ATTR_STD_FACE_NORMAL);
  mesh->add_face_normals();
  mesh->add_vertex_normals();
  mesh->attributes.remove(ATTR_STD_FACE_NORMAL);

  /* texture coords still needed */
}

static void ExportCurveSegments(Scene *scene, Mesh *mesh, ParticleCurveData *CData)
{
  int num_keys = 0;
  int num_curves = 0;
  int human_countable_num_curves = 0;

  if(mesh->num_curves())
      return;

  Attribute *attr_intercept = NULL;
  Attribute *attr_random = NULL;
  Attribute *attr_index = NULL;
  Attribute *attr_count = NULL;
  Attribute *attr_length = NULL;
  Attribute *attr_key = NULL;
  Attribute *attr_value = NULL;

  if (mesh->need_attribute(scene, ATTR_STD_CURVE_INTERCEPT))
    attr_intercept = mesh->curve_attributes.add(ATTR_STD_CURVE_INTERCEPT);
  if (mesh->need_attribute(scene, ATTR_STD_CURVE_RANDOM))
    attr_random = mesh->curve_attributes.add(ATTR_STD_CURVE_RANDOM);
  if(mesh->need_attribute(scene, ATTR_STD_CURVE_INDEX))
      attr_index = mesh->curve_attributes.add(ATTR_STD_CURVE_INDEX);
  if(mesh->need_attribute(scene, ATTR_STD_CURVE_COUNT))
      attr_count = mesh->curve_attributes.add(ATTR_STD_CURVE_COUNT);
  if(mesh->need_attribute(scene, ATTR_STD_CURVE_LENGTH))
      attr_length = mesh->curve_attributes.add(ATTR_STD_CURVE_LENGTH);
  if(mesh->need_attribute(scene, ATTR_STD_CURVE_KEY))
      attr_key = mesh->curve_attributes.add(ATTR_STD_CURVE_KEY);
  if(mesh->need_attribute(scene, ATTR_STD_CURVE_VALUE))
      attr_value = mesh->curve_attributes.add(ATTR_STD_CURVE_VALUE);

  /* compute and reserve size of arrays */
  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) 
  {
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) 
    {
      num_keys += CData->curve_keynum[curve];
      num_curves++;
      human_countable_num_curves++;
    }
  }

  if (num_curves > 0) {
    VLOG(1) << "Exporting curve segments for mesh " << mesh->name;
  }

  mesh->reserve_curves(mesh->num_curves() + num_curves, mesh->curve_keys.size() + num_keys);

  num_keys = 0;
  num_curves = 0;

  /* actually export */
  bool render_as_hair = false;
  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) 
  {
    if (CData->render_as_hair.size() > sys)
      render_as_hair = CData->render_as_hair[sys];
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) 
    {
      size_t num_curve_keys = 0;

      for (int curvekey = CData->curve_firstkey[curve];
           curvekey < CData->curve_firstkey[curve] + CData->curve_keynum[curve];
           curvekey++) 
      {
        const float3 ickey_loc = CData->curvekey_co[curvekey];
        const float curve_time = CData->curvekey_time[curvekey];
        const float curve_length = CData->curve_length[curve];
        const float time = (curve_length > 0.0f) ? curve_time / curve_length : 0.0f;
        float radius = 0.0f;
        if (render_as_hair) 
        {
          radius = CData->curvekey_radius[curvekey];
        }
        else 
        {
          radius = shaperadius(
          CData->psys_shape[sys], CData->psys_rootradius[sys], CData->psys_tipradius[sys], time);
          if (CData->psys_closetip[sys] &&
              (curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1))
            radius = 0.0f;
        }
        mesh->add_curve_key(ickey_loc, radius);
        if (attr_intercept)
          attr_intercept->add(time);

        if(attr_value != NULL)
          attr_value->add(CData->curvekey_value[curvekey]);

        if(attr_key != NULL)
          attr_key->add(CData->curvekey_key[curvekey]);

        num_curve_keys++;
      }

      if (attr_random != NULL) {
        attr_random->add(hash_uint2_to_float(num_curves, 0));
      }

      if (attr_index != NULL)
        attr_index->add(static_cast< float >(curve));

      if (attr_count != NULL)
        attr_count->add(static_cast< float >(human_countable_num_curves));

      if (attr_length != NULL)
        attr_length->add(CData->curve_length[curve]);

      int shader;
      // Variable "curve" is actually referring to a spline within a curve object
      if (render_as_hair)
        shader = CData->curve_shader[curve];
      else 
        shader = CData->psys_shader[sys];

      mesh->add_curve(num_keys, shader);
      num_keys += num_curve_keys;
      num_curves++;
    }
  }

  /* check allocation */
  if ((mesh->curve_keys.size() != num_keys) || (mesh->num_curves() != num_curves)) {
    VLOG(1) << "Allocation failed, clearing data";
    mesh->clear();
  }
}

static float4 CurveSegmentMotionCV(ParticleCurveData *CData, int sys, int curve, int curvekey)
{
  const float3 ickey_loc = CData->curvekey_co[curvekey];
  const float curve_time = CData->curvekey_time[curvekey];
  const float curve_length = CData->curve_length[curve];
  float time = (curve_length > 0.0f) ? curve_time / curve_length : 0.0f;
  float radius;
  bool render_as_hair = false;
  if (CData->render_as_hair.size() > sys)
    render_as_hair = CData->render_as_hair[sys];
  if (render_as_hair) {
      radius = CData->curvekey_radius[curvekey];
  }
  else {
    radius = shaperadius(
      CData->psys_shape[sys], CData->psys_rootradius[sys], CData->psys_tipradius[sys], time);

    if (CData->psys_closetip[sys] &&
        (curvekey == CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1))
      radius = 0.0f;
  }
  /* curve motion keys store both position and radius in float4 */
  float4 mP = float3_to_float4(ickey_loc);
  mP.w = radius;
  return mP;
}

static float4 LerpCurveSegmentMotionCV(ParticleCurveData *CData, int sys, int curve, float step)
{
  assert(step >= 0.0f);
  assert(step <= 1.0f);
  const int first_curve_key = CData->curve_firstkey[curve];
  const float curve_key_f = step * (CData->curve_keynum[curve] - 1);
  int curvekey = (int)floorf(curve_key_f);
  const float remainder = curve_key_f - curvekey;
  if (remainder == 0.0f) {
    return CurveSegmentMotionCV(CData, sys, curve, first_curve_key + curvekey);
  }
  int curvekey2 = curvekey + 1;
  if (curvekey2 >= (CData->curve_keynum[curve] - 1)) {
    curvekey2 = (CData->curve_keynum[curve] - 1);
    curvekey = curvekey2 - 1;
  }
  const float4 mP = CurveSegmentMotionCV(CData, sys, curve, first_curve_key + curvekey);
  const float4 mP2 = CurveSegmentMotionCV(CData, sys, curve, first_curve_key + curvekey2);
  return lerp(mP, mP2, remainder);
}

static void ExportCurveSegmentsMotion(Mesh *mesh, ParticleCurveData *CData, int motion_step)
{
  VLOG(1) << "Exporting curve motion segments for mesh " << mesh->name << ", motion step "
          << motion_step;

  /* find attribute */
  Attribute *attr_mP = mesh->curve_attributes.find(ATTR_STD_MOTION_VERTEX_POSITION);
  bool new_attribute = false;

  /* add new attribute if it doesn't exist already */
  if (!attr_mP) {
    VLOG(1) << "Creating new motion vertex position attribute";
    attr_mP = mesh->curve_attributes.add(ATTR_STD_MOTION_VERTEX_POSITION);
    new_attribute = true;
  }

  /* export motion vectors for curve keys */
  size_t numkeys = mesh->curve_keys.size();
  float4 *mP = attr_mP->data_float4() + motion_step * numkeys;
  bool have_motion = false;
  int i = 0;
  int num_curves = 0;

  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) {
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) {
      /* Curve lengths may not match! Curves can be clipped. */
      int curve_key_end = (num_curves + 1 < (int)mesh->curve_first_key.size() ?
                               mesh->curve_first_key[num_curves + 1] :
                               (int)mesh->curve_keys.size());
      const int num_center_curve_keys = curve_key_end - mesh->curve_first_key[num_curves];
      const int is_num_keys_different = CData->curve_keynum[curve] - num_center_curve_keys;

      if (!is_num_keys_different) {
        for (int curvekey = CData->curve_firstkey[curve];
             curvekey < CData->curve_firstkey[curve] + CData->curve_keynum[curve];
             curvekey++) {
          if (i < mesh->curve_keys.size()) {
            mP[i] = CurveSegmentMotionCV(CData, sys, curve, curvekey);
            if (!have_motion) {
              /* unlike mesh coordinates, these tend to be slightly different
               * between frames due to particle transforms into/out of object
               * space, so we use an epsilon to detect actual changes */
              float4 curve_key = float3_to_float4(mesh->curve_keys[i]);
              curve_key.w = mesh->curve_radius[i];
              if (len_squared(mP[i] - curve_key) > 1e-5f * 1e-5f)
                have_motion = true;
            }
          }
          i++;
        }
      }
      else {
        /* Number of keys has changed. Generate an interpolated version
         * to preserve motion blur. */
        const float step_size = num_center_curve_keys > 1 ? 1.0f / (num_center_curve_keys - 1) :
                                                            0.0f;
        for (int step_index = 0; step_index < num_center_curve_keys; ++step_index) {
          const float step = step_index * step_size;
          mP[i] = LerpCurveSegmentMotionCV(CData, sys, curve, step);
          i++;
        }
        have_motion = true;
      }
      num_curves++;
    }
  }

  /* in case of new attribute, we verify if there really was any motion */
  if (new_attribute) {
    if (i != numkeys || !have_motion) {
      /* No motion or hair "topology" changed, remove attributes again. */
      if (i != numkeys) {
        VLOG(1) << "Hair topology changed, removing attribute.";
      }
      else {
        VLOG(1) << "No motion, removing attribute.";
      }
      mesh->curve_attributes.remove(ATTR_STD_MOTION_VERTEX_POSITION);
    }
    else if (motion_step > 0) {
      VLOG(1) << "Filling in new motion vertex position for motion_step " << motion_step;
      /* motion, fill up previous steps that we might have skipped because
       * they had no motion, but we need them anyway now */
      for (int step = 0; step < motion_step; step++) {
        float4 *mP = attr_mP->data_float4() + step * numkeys;

        for (int key = 0; key < numkeys; key++) {
          mP[key] = float3_to_float4(mesh->curve_keys[key]);
          mP[key].w = mesh->curve_radius[key];
        }
      }
    }
  }
}

static void ExportCurveTriangleUV(ParticleCurveData *CData,
                                  int vert_offset,
                                  int resol,
                                  float2 *uvdata)
{
  if (uvdata == NULL)
    return;
  int vertexindex = vert_offset;

  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) {
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) {
      for (int curvekey = CData->curve_firstkey[curve];
           curvekey < CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1;
           curvekey++) {
        for (int section = 0; section < resol; section++) {
          uvdata[vertexindex] = CData->curve_uv[curve];
          vertexindex++;
          uvdata[vertexindex] = CData->curve_uv[curve];
          vertexindex++;
          uvdata[vertexindex] = CData->curve_uv[curve];
          vertexindex++;
          uvdata[vertexindex] = CData->curve_uv[curve];
          vertexindex++;
          uvdata[vertexindex] = CData->curve_uv[curve];
          vertexindex++;
          uvdata[vertexindex] = CData->curve_uv[curve];
          vertexindex++;
        }
      }
    }
  }
}

static void ExportCurveTriangleVcol(ParticleCurveData *CData,
                                    int vert_offset,
                                    int resol,
                                    uchar4 *cdata)
{
  if (cdata == NULL)
    return;

  int vertexindex = vert_offset;

  for (int sys = 0; sys < CData->psys_firstcurve.size(); sys++) {
    for (int curve = CData->psys_firstcurve[sys];
         curve < CData->psys_firstcurve[sys] + CData->psys_curvenum[sys];
         curve++) {
      for (int curvekey = CData->curve_firstkey[curve];
           curvekey < CData->curve_firstkey[curve] + CData->curve_keynum[curve] - 1;
           curvekey++) {
        for (int section = 0; section < resol; section++) {
          /* Encode vertex color using the sRGB curve. */
          cdata[vertexindex] = color_float_to_byte(
              color_srgb_to_linear_v3(CData->curve_vcol[curve]));
          vertexindex++;
          cdata[vertexindex] = color_float_to_byte(
              color_srgb_to_linear_v3(CData->curve_vcol[curve]));
          vertexindex++;
          cdata[vertexindex] = color_float_to_byte(
              color_srgb_to_linear_v3(CData->curve_vcol[curve]));
          vertexindex++;
          cdata[vertexindex] = color_float_to_byte(
              color_srgb_to_linear_v3(CData->curve_vcol[curve]));
          vertexindex++;
          cdata[vertexindex] = color_float_to_byte(
              color_srgb_to_linear_v3(CData->curve_vcol[curve]));
          vertexindex++;
          cdata[vertexindex] = color_float_to_byte(
              color_srgb_to_linear_v3(CData->curve_vcol[curve]));
          vertexindex++;
        }
      }
    }
  }
}

/* Hair Curve Sync */

void BlenderSync::sync_curve_settings()
{
  PointerRNA csscene = RNA_pointer_get(&b_scene.ptr, "cycles_curves");

  CurveSystemManager *curve_system_manager = scene->curve_system_manager;
  CurveSystemManager prev_curve_system_manager = *curve_system_manager;

  curve_system_manager->use_curves = get_boolean(csscene, "use_curves");

  curve_system_manager->primitive = (CurvePrimitiveType)get_enum(
      csscene, "primitive", CURVE_NUM_PRIMITIVE_TYPES, CURVE_LINE_SEGMENTS);
  curve_system_manager->curve_shape = (CurveShapeType)get_enum(
      csscene, "shape", CURVE_NUM_SHAPE_TYPES, CURVE_THICK);
  curve_system_manager->resolution = get_int(csscene, "resolution");
  curve_system_manager->subdivisions = get_int(csscene, "subdivisions");
  curve_system_manager->use_backfacing = !get_boolean(csscene, "cull_backfacing");

  /* Triangles */
  if (curve_system_manager->primitive == CURVE_TRIANGLES) {
    /* camera facing planes */
    if (curve_system_manager->curve_shape == CURVE_RIBBON) {
      curve_system_manager->triangle_method = CURVE_CAMERA_TRIANGLES;
      curve_system_manager->resolution = 1;
    }
    else if (curve_system_manager->curve_shape == CURVE_THICK) {
      curve_system_manager->triangle_method = CURVE_TESSELATED_TRIANGLES;
    }
  }
  /* Line Segments */
  else if (curve_system_manager->primitive == CURVE_LINE_SEGMENTS) {
    if (curve_system_manager->curve_shape == CURVE_RIBBON) {
      /* tangent shading */
      curve_system_manager->line_method = CURVE_UNCORRECTED;
      curve_system_manager->use_encasing = true;
      curve_system_manager->use_backfacing = false;
      curve_system_manager->use_tangent_normal_geometry = true;
    }
    else if (curve_system_manager->curve_shape == CURVE_THICK) {
      curve_system_manager->line_method = CURVE_ACCURATE;
      curve_system_manager->use_encasing = false;
      curve_system_manager->use_tangent_normal_geometry = false;
    }
  }
  /* Curve Segments */
  else if (curve_system_manager->primitive == CURVE_SEGMENTS) {
    if (curve_system_manager->curve_shape == CURVE_RIBBON) {
      curve_system_manager->primitive = CURVE_RIBBONS;
      curve_system_manager->use_backfacing = false;
    }
  }

  if (curve_system_manager->modified_mesh(prev_curve_system_manager)) {
    BL::BlendData::objects_iterator b_ob;

    for (b_data.objects.begin(b_ob); b_ob != b_data.objects.end(); ++b_ob) {
      if (object_is_mesh(*b_ob)) {
        BL::Object::particle_systems_iterator b_psys;
        for (b_ob->particle_systems.begin(b_psys); b_psys != b_ob->particle_systems.end();
             ++b_psys) {
          if ((b_psys->settings().render_type() == BL::ParticleSettings::render_type_PATH) &&
              (b_psys->settings().type() == BL::ParticleSettings::type_HAIR)) {
            BL::ID key = BKE_object_is_modified(*b_ob) ? *b_ob : b_ob->data();
            mesh_map.set_recalc(key);
            object_map.set_recalc(*b_ob);
          }
        }
      }
    }
  }

  if (curve_system_manager->modified(prev_curve_system_manager))
    curve_system_manager->tag_update(scene);
}

void BlenderSync::sync_curves(
    Mesh *mesh, BL::Mesh &b_mesh, BL::Object &b_ob, bool motion, int motion_step)
{
  if (!motion) {
    /* Clear stored curve data */
    mesh->curve_keys.clear();
    mesh->curve_radius.clear();
    mesh->curve_first_key.clear();
    mesh->curve_shader.clear();
    mesh->curve_attributes.clear();
  }

  /* obtain general settings */
  const bool use_curves = scene->curve_system_manager->use_curves;

  if (!(use_curves && b_ob.mode() != b_ob.mode_PARTICLE_EDIT && b_ob.mode() != b_ob.mode_EDIT)) {
    if (!motion)
      mesh->compute_bounds();
    return;
  }

  const int primitive = scene->curve_system_manager->primitive;
  const int triangle_method = scene->curve_system_manager->triangle_method;
  const int resolution = scene->curve_system_manager->resolution;
  const size_t vert_num = mesh->verts.size();
  const size_t tri_num = mesh->num_triangles();
  int used_res = 1;

  /* extract particle hair data - should be combined with connecting to mesh later*/

  BL::ID b_ob_data = b_ob.data();
  bool render_as_hair = false;

  // [Nicolas Antille] : cycles_curves is here a property of the Curve data 
  if(!b_ob_data || b_ob_data.is_a(&RNA_Curve)) {
    PointerRNA data = RNA_pointer_get(&b_ob_data.ptr, "cycles_curves");
    render_as_hair = get_boolean(data, "render_as_hair");
  }

  ParticleCurveData CData;

  if (render_as_hair)
    ObtainCurveData(mesh, &b_mesh, &b_ob, &CData, !preview);
  else
    ObtainCacheParticleData(mesh, &b_mesh, &b_ob, &CData, !preview);

  /* add hair geometry to mesh */
  if (primitive == CURVE_TRIANGLES) {
    if (triangle_method == CURVE_CAMERA_TRIANGLES) {
      /* obtain camera parameters */
      float3 RotCam;
      Camera *camera = scene->camera;
      Transform &ctfm = camera->matrix;
      if (camera->type == CAMERA_ORTHOGRAPHIC) {
        RotCam = -make_float3(ctfm.x.z, ctfm.y.z, ctfm.z.z);
      }
      else {
        Transform tfm = get_transform(b_ob.matrix_world());
        Transform itfm = transform_quick_inverse(tfm);
        RotCam = transform_point(&itfm, make_float3(ctfm.x.w, ctfm.y.w, ctfm.z.w));
      }
      bool is_ortho = camera->type == CAMERA_ORTHOGRAPHIC;
      ExportCurveTrianglePlanes(mesh, &CData, RotCam, is_ortho);
    }
    else {
      ExportCurveTriangleGeometry(mesh, &CData, resolution);
      used_res = resolution;
    }
  }
  else {
    if (motion)
      ExportCurveSegmentsMotion(mesh, &CData, motion_step);
    else
      ExportCurveSegments(scene, mesh, &CData);
  }

  /* generated coordinates from first key. we should ideally get this from
   * blender to handle deforming objects */
  if (!motion && !render_as_hair) {
    if (mesh->need_attribute(scene, ATTR_STD_GENERATED)) {
      float3 loc, size;
      mesh_texture_space(b_mesh, loc, size);

      if (primitive == CURVE_TRIANGLES) {
        Attribute *attr_generated = mesh->attributes.add(ATTR_STD_GENERATED);
        float3 *generated = attr_generated->data_float3();

        for (size_t i = vert_num; i < mesh->verts.size(); i++)
          generated[i] = mesh->verts[i] * size - loc;
      }
      else {
        Attribute *attr_generated = mesh->curve_attributes.add(ATTR_STD_GENERATED);
        float3 *generated = attr_generated->data_float3();

        for (size_t i = 0; i < mesh->num_curves(); i++) {
          float3 co = mesh->curve_keys[mesh->get_curve(i).first_key];
          generated[i] = co * size - loc;
        }
      }
    }
  }

  /* create vertex color attributes */
  if (!motion && !render_as_hair) {
    BL::Mesh::vertex_colors_iterator l;
    int vcol_num = 0;

    for (b_mesh.vertex_colors.begin(l); l != b_mesh.vertex_colors.end(); ++l, vcol_num++) {
      if (!mesh->need_attribute(scene, ustring(l->name().c_str())))
        continue;

      ObtainCacheParticleVcol(mesh, &b_mesh, &b_ob, &CData, !preview, vcol_num);

      if (primitive == CURVE_TRIANGLES) {
        Attribute *attr_vcol = mesh->attributes.add(
            ustring(l->name().c_str()), TypeDesc::TypeColor, ATTR_ELEMENT_CORNER_BYTE);

        uchar4 *cdata = attr_vcol->data_uchar4();

        ExportCurveTriangleVcol(&CData, tri_num * 3, used_res, cdata);
      }
      else {
        Attribute *attr_vcol = mesh->curve_attributes.add(
            ustring(l->name().c_str()), TypeDesc::TypeColor, ATTR_ELEMENT_CURVE);

        float3 *fdata = attr_vcol->data_float3();

        if (fdata) {
          size_t i = 0;

          /* Encode vertex color using the sRGB curve. */
          for (size_t curve = 0; curve < CData.curve_vcol.size(); curve++) {
            fdata[i++] = color_srgb_to_linear_v3(CData.curve_vcol[curve]);
          }
        }
      }
    }
  }

  /* create UV attributes */
  if (!motion && !render_as_hair) {
    BL::Mesh::uv_layers_iterator l;
    int uv_num = 0;

    for (b_mesh.uv_layers.begin(l); l != b_mesh.uv_layers.end(); ++l, uv_num++) {
      bool active_render = l->active_render();
      AttributeStandard std = (active_render) ? ATTR_STD_UV : ATTR_STD_NONE;
      ustring name = ustring(l->name().c_str());

      /* UV map */
      if (mesh->need_attribute(scene, name) || mesh->need_attribute(scene, std)) {
        Attribute *attr_uv;

        ObtainCacheParticleUV(mesh, &b_mesh, &b_ob, &CData, !preview, uv_num);

        if (primitive == CURVE_TRIANGLES) {
          if (active_render)
            attr_uv = mesh->attributes.add(std, name);
          else
            attr_uv = mesh->attributes.add(name, TypeFloat2, ATTR_ELEMENT_CORNER);

          float2 *uv = attr_uv->data_float2();

          ExportCurveTriangleUV(&CData, tri_num * 3, used_res, uv);
        }
        else {
          if (active_render)
            attr_uv = mesh->curve_attributes.add(std, name);
          else
            attr_uv = mesh->curve_attributes.add(name, TypeFloat2, ATTR_ELEMENT_CURVE);

          float2 *uv = attr_uv->data_float2();

          if (uv) {
            size_t i = 0;

            for (size_t curve = 0; curve < CData.curve_uv.size(); curve++) {
              uv[i++] = CData.curve_uv[curve];
            }
          }
        }
      }
    }
  }

  mesh->compute_bounds();
}

CCL_NAMESPACE_END
