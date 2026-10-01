/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license in the file COPYING
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file COPYING.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2022 Saso Kiselkov. All rights reserved.
 */





#include "BpGeometryCompat.h"
namespace bpgeometry {
using std::isnan;
#define    SEG_TURN_MULT        0.9    /* leave 10% for oversteer */
#define    MIN_TURN_RADIUS        1.5    /* in case the aircraft is tiny */
#define    MIN_STEERING_ARM_LEN    4    /* meters */
#define    MAX_OFF_PATH_ANGLE    20    /* degrees */
#define    OFF_PATH_CORR_ANGLE    35    /* degrees */
#define    STEERING_SENSITIVE    90    /* degrees */

#define    MIN_SEG_LEN        0.1    /* meters */

#define    STEER_GATE(x, g)    MIN(MAX((x), -g), g)

#define    ROUTE_DIST_LIM        30    /* meters */
#define    ROUTE_HDG_LIM        10    /* degrees */
#define    ROUTE_TABLE_DIRS    bp_xpdir, "Output", "caches"
#define    ROUTE_TABLE_FILENAME    "BetterPushback_routes.dat"

/*
 * When constructing the oblique case, rounding errors can cause us to
 * compute a case as usable, but the construction will fail (because the
 * actual required turn radius will have dropped below the minimum). So
 * when invoking the oblique case, we inflate our minimum radius by this
 * factor to work around the rounding errors.
 */
#define    OBLIQUE_RADIUS_FACT    1.02

#define    STRAIGHT_SEG_ANGLE_LIM    1

/* Turns on aggressive debug logging. */
/*#define	DRIVING_DEBUG_LOGGING*/

static int compute_segs_impl(const vehicle_t *veh, vect2_t start_pos,
                             double start_hdg, vect2_t end_pos, double end_hdg, list_t *segs,
                             bool_t recurse);

static int
construct_segs_oblique(const vehicle_t *veh, vect2_t start_pos,
                       double start_hdg, vect2_t midpt, double mid_hdg, vect2_t end_pos,
                       double end_hdg, list_t *segs) {
    int n1, n2;

    n1 = compute_segs_impl(veh, start_pos, start_hdg, midpt, mid_hdg,
                           segs, B_FALSE);
    if (n1 == -1)
        return (-1);
    n2 = compute_segs_impl(veh, midpt, mid_hdg, end_pos, end_hdg,
                           segs, B_FALSE);
    if (n2 == -1) {
        for (int i = 0; i < n1; i++) {
            seg_t *seg = list_remove_tail(segs);
            free(seg);
        }
        return (-1);
    }

    return (n1 + n2);
}

static int
compute_segs_oblique(const vehicle_t *veh, vect2_t start_pos,
                     double start_hdg, vect2_t end_pos, double end_hdg, list_t *segs,
                     bool_t backward, double radius) {
    const vect2_t d1 = hdg2dir(start_hdg), d2 = hdg2dir(end_hdg);
    const vect2_t sv = vect2_scmul(d1, backward ? -1e10 : 1e10);
    const vect2_t ev = vect2_scmul(d2, backward ? 1e10 : -1e10);

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            vect2_t c1 = vect2_add(start_pos,
                                   vect2_scmul(vect2_norm(d1, i), radius));
            vect2_t c2 = vect2_add(end_pos,
                                   vect2_scmul(vect2_norm(d2, j), radius));
            vect2_t s = vect2_mean(c1, c2);
            vect2_t s2c1 = vect2_sub(c1, s);
            vect2_t s2c2 = vect2_sub(c2, s);
            double a;

            if (!IS_NULL_VECT(vect2vect_isect(s2c1, s, sv,
                                              start_pos, B_TRUE)) ||
                !IS_NULL_VECT(vect2vect_isect(s2c2, s, ev,
                                              end_pos, B_TRUE)))
                continue;

            a = RAD2DEG(asin(radius / vect2_abs(s2c1)));
            if (isnan(a))
                continue;

            if (!IS_NULL_VECT(vect2vect_isect(vect2_set_abs(
                                                      vect2_rot(s2c1, a), 1e10), s, sv, start_pos,
                                              B_TRUE)) &&
                !IS_NULL_VECT(vect2vect_isect(vect2_set_abs(
                                                      vect2_rot(s2c2, a), 1e10), s, ev, end_pos,
                                              B_TRUE))) {
                int n = construct_segs_oblique(veh, start_pos,
                                               start_hdg, s, dir2hdg(vect2_rot(backward ?
                                                                               s2c1 : s2c2, a)), end_pos, end_hdg,
                                               segs);
                if (n != -1)
                    return (n);
            }

            if (!IS_NULL_VECT(vect2vect_isect(vect2_set_abs(
                                                      vect2_rot(s2c1, -a), 1e10), s, sv, start_pos,
                                              B_TRUE)) &&
                !IS_NULL_VECT(vect2vect_isect(vect2_set_abs(
                                                      vect2_rot(s2c2, -a), 1e10), s, ev, end_pos,
                                              B_TRUE))) {
                int n = construct_segs_oblique(veh, start_pos,
                                               start_hdg, s, dir2hdg(vect2_rot(backward ?
                                                                               s2c1 : s2c2, -a)), end_pos, end_hdg,
                                               segs);
                if (n != -1)
                    return (n);
            }
        }
    }

    return (-1);
}

static int
compute_segs_impl(const vehicle_t *veh, vect2_t start_pos, double start_hdg,
                  vect2_t end_pos, double end_hdg, list_t *segs, bool_t recurse) {
    seg_t *s1, *s2;
    vect2_t turn_edge, s1_v, s2_v, s2e_v;
    double rhdg, min_radius, l1, l2, x, a, r;
    bool_t backward;

    /* If the start & end positions overlap, no operation is required */
    if (vect2_dist(start_pos, end_pos) < MIN_SEG_LEN) {
        if (ABS(start_hdg - end_hdg) < STRAIGHT_SEG_ANGLE_LIM)
            return (0);
        else
            return (-1);
    }
    s2e_v = vect2_sub(end_pos, start_pos);
    rhdg = rel_hdg(start_hdg, dir2hdg(s2e_v));
    backward = (fabs(rhdg) > 90);

    /*
     * Compute minimum radius using less than max_steer (hence
     * SEG_TURN_MULT), to allow for some oversteering correction.
     * Also limit the radius to something sensible (MIN_TURN_RADIUS).
     */
    min_radius = MAX(tan(DEG2RAD(90 - (veh->max_steer * SEG_TURN_MULT))) *
                     veh->wheelbase, MIN_TURN_RADIUS);

    /*
     * If the amount of heading change is tiny, just project the desired
     * end point onto a straight vector from our starting position and
     * construct a single straight segment to reach that point.
     */
    if (fabs(start_hdg - end_hdg) < STRAIGHT_SEG_ANGLE_LIM &&
        (ABS(rhdg) < STRAIGHT_SEG_ANGLE_LIM ||
         ABS(rhdg) > 180 - STRAIGHT_SEG_ANGLE_LIM)) {
        vect2_t dir_v = hdg2dir(start_hdg + (backward ? 180 : 0));
        double len = vect2_dotprod(dir_v, s2e_v);

        end_pos = vect2_add(vect2_set_abs(dir_v, len), start_pos);

        s1 = safe_calloc(1, sizeof(*s1));
        s1->type = SEG_TYPE_STRAIGHT;
        s1->start_pos = start_pos;
        s1->start_hdg = start_hdg;
        s1->end_pos = end_pos;
        s1->end_hdg = end_hdg;
        s1->backward = backward;
        s1->len = len;

        list_insert_tail(segs, s1);

        return (1);
    }

    s1_v = vect2_scmul(hdg2dir(start_hdg), 1e10);
    if (backward)
        s1_v = vect2_neg(s1_v);
    s2_v = vect2_scmul(hdg2dir(end_hdg), 1e10);
    if (!backward)
        s2_v = vect2_neg(s2_v);

    turn_edge = vect2vect_isect(s1_v, start_pos, s2_v, end_pos, B_TRUE);
    if (IS_NULL_VECT(turn_edge)) {
        if (recurse) {
            return (compute_segs_oblique(veh, start_pos, start_hdg,
                                         end_pos, end_hdg, segs, backward,
                                         min_radius * OBLIQUE_RADIUS_FACT));
        } else {
            return (-1);
        }
    }

    l1 = vect2_dist(turn_edge, start_pos);
    l2 = vect2_dist(turn_edge, end_pos);
    x = MIN(l1, l2);
    l1 -= x;
    l2 -= x;

    a = (180 - ABS(rel_hdg(start_hdg, end_hdg)));
    r = x * tan(DEG2RAD(a / 2));
    if (r < min_radius) {
        if (recurse)
            return (compute_segs_oblique(veh, start_pos, start_hdg,
                                         end_pos, end_hdg, segs, backward,
                                         min_radius * OBLIQUE_RADIUS_FACT));
        else
            return (-1);
    }
    if (l1 == 0) {
        /* No initial straight segment */
        s2 = safe_calloc(1, sizeof(*s2));
        s2->type = SEG_TYPE_STRAIGHT;
        s2->start_pos = vect2_add(end_pos, vect2_set_abs(s2_v, l2));
        s2->start_hdg = end_hdg;
        s2->end_pos = end_pos;
        s2->end_hdg = end_hdg;
        s2->backward = backward;
        s2->len = l2;

        s1 = safe_calloc(1, sizeof(*s1));
        s1->type = SEG_TYPE_TURN;
        s1->start_pos = start_pos;
        s1->start_hdg = start_hdg;
        s1->end_pos = s2->start_pos;
        s1->end_hdg = s2->start_hdg;
        s1->backward = backward;
        s1->turn.r = r;
        s1->turn.right = (rhdg >= 0);
    } else {
        /* No final straight segment */
        s1 = safe_calloc(1, sizeof(*s1));
        s1->type = SEG_TYPE_STRAIGHT;
        s1->start_pos = start_pos;
        s1->start_hdg = start_hdg;
        s1->end_pos = vect2_add(start_pos, vect2_set_abs(s1_v, l1));
        s1->end_hdg = start_hdg;
        s1->backward = backward;
        s1->len = l1;

        s2 = safe_calloc(1, sizeof(*s2));
        s2->type = SEG_TYPE_TURN;
        s2->start_pos = s1->end_pos;
        s2->start_hdg = s1->end_hdg;
        s2->end_pos = end_pos;
        s2->end_hdg = end_hdg;
        s2->backward = backward;
        s2->turn.r = r;
        s2->turn.right = (rhdg >= 0);
    }

    list_insert_tail(segs, s1);
    list_insert_tail(segs, s2);

    return (2);
}

int
compute_segs(const vehicle_t *veh, vect2_t start_pos, double start_hdg,
             vect2_t end_pos, double end_hdg, list_t *segs) {
    return (compute_segs_impl(veh, start_pos, start_hdg, end_pos,
                              end_hdg, segs, B_TRUE));
}


} // namespace bpgeometry
