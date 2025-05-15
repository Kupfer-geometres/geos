/**********************************************************************
 *
 * GEOS - Geometry Engine Open Source
 * http://geos.osgeo.org
 *
 * Copyright (C) 2024 Your Name
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the GNU Lesser General Public Licence as published
 * by the Free Software Foundation.
 * See the COPYING file for more information.
 *
 **********************************************************************/

 #pragma once

 #include <geos/geom/Geometry.h>
 #include <geos/geom/CoordinateSequence.h>
 #include <geos/geom/Coordinate.h>
 #include <geos/geom/CircularArc.h>
 #include <memory>
 #include <vector>
 
 // Forward declarations
 namespace geos {
 namespace geom {
     class CircularString;
     class LineString;
     class GeometryFactory;
     class CurvePolygon;
     class Curve;
 }
 }
 
 namespace geos {
 namespace operation {
 namespace overlayng {
 
 /**
  * Utilities for handling curved geometries in overlay operations
  */
 class CurveOverlayUtil {
 public:
     /**
      * Checks if a geometry contains curved components
      * @param geom the geometry to check
      * @return true if the geometry contains curves, false otherwise
      */
     static bool hasCurvedComponents(const geom::Geometry* geom);
     
     /**
      * Performs overlay operations on geometries with curved components
      * @param geom0 the first geometry
      * @param geom1 the second geometry  
      * @param opCode the overlay operation type
      * @return the result geometry
      */
     static std::unique_ptr<geom::Geometry> overlayCurves(
         const geom::Geometry* geom0, 
         const geom::Geometry* geom1, 
         int opCode);
         
 private:
     /**
      * Handles intersection between a CircularString and a LineString
      */
     static std::unique_ptr<geom::Geometry> intersectCircularStringLineString(
         const geom::CircularString* cs,
         const geom::LineString* ls);
         
     /**
      * Handles intersection between two CircularStrings
      */
     static std::unique_ptr<geom::Geometry> intersectCircularStrings(
         const geom::CircularString* cs1,
         const geom::CircularString* cs2);
         
     /**
      * Handles overlay operations for intersection
      */
     static std::unique_ptr<geom::Geometry> overlayCurvesIntersection(
         const geom::Geometry* geom0,
         const geom::Geometry* geom1,
         const geom::GeometryFactory* factory);
         
     /**
      * Handles overlay operations for union
      */
     static std::unique_ptr<geom::Geometry> overlayCurvesUnion(
         const geom::Geometry* geom0,
         const geom::Geometry* geom1,
         const geom::GeometryFactory* factory);
         
     /**
      * Handles overlay operations for difference
      */
     static std::unique_ptr<geom::Geometry> overlayCurvesDifference(
         const geom::Geometry* geom0,
         const geom::Geometry* geom1,
         const geom::GeometryFactory* factory);
         
     /**
      * Handles overlay operations for symmetric difference
      */
     static std::unique_ptr<geom::Geometry> overlayCurvesSymDifference(
         const geom::Geometry* geom0,
         const geom::Geometry* geom1,
         const geom::GeometryFactory* factory);
         
     /**
      * Finds intersection between a circular arc and a line segment
      */
     static std::vector<geom::Coordinate> intersectArcSegment(
         const geom::CircularArc& arc,
         const geom::Coordinate& segStart,
         const geom::Coordinate& segEnd);
         
     /**
      * Handles union between two CircularStrings
      */
     static std::unique_ptr<geom::Geometry> unionCircularStrings(
         const geom::CircularString* cs1,
         const geom::CircularString* cs2,
         const geom::GeometryFactory* factory);
         
     /**
      * Handles union between two CurvePolygons
      */
     static std::unique_ptr<geom::Geometry> unionCurvePolygons(
         const geom::CurvePolygon* cp1,
         const geom::CurvePolygon* cp2,
         const geom::GeometryFactory* factory);
         
     /**
      * Vérifie si un point est sur un arc
      */
     static bool isPointOnArc(
         const geom::CircularArc& arc,
         const geom::Coordinate& point,
         double epsilon = 1e-9);
         
     /**
      * Divise un arc en deux au point donné
      */
     static std::vector<std::unique_ptr<geom::CircularString>> splitArc(
         const geom::CircularString* arc,
         const geom::Coordinate& splitPoint,
         const geom::GeometryFactory* factory);
         
     /**
      * Trouve les intersections entre deux arcs
      */
     static std::vector<geom::Coordinate> intersectArcs(
         const geom::CircularArc& arc1,
         const geom::CircularArc& arc2);
         
     /**
      * Extrait tous les segments/arcs d'une courbe
      */
     static std::vector<std::unique_ptr<geom::Curve>> extractCurveSegments(
         const geom::Curve* curve,
         const geom::GeometryFactory* factory);
         
     /**
      * Obtient le point milieu d'une courbe
      */
     static geom::Coordinate getMidPoint(const geom::Curve* curve);
         
     /**
      * Vérifie si un point est dans un polygone courbe
      */
     static bool isPointInPolygon(
         const geom::Coordinate& point,
         const geom::CurvePolygon* polygon);
         
     /**
      * Vérifie si un segment est sur la bordure d'un polygone
      */
     static bool isSegmentOnBoundary(
         const geom::Curve* segment,
         const geom::CurvePolygon* polygon);
         
     /**
      * Construit un anneau fermé à partir de segments
      */
     static std::unique_ptr<geom::Curve> buildRingFromSegments(
         std::vector<std::unique_ptr<geom::Curve>>& segments,
         const geom::GeometryFactory* factory);
         
     /**
      * Compare deux segments pour l'égalité
      */
     static bool segmentsAreEqual(
         const geom::Curve* seg1,
         const geom::Curve* seg2);
         
     /**
      * Obtient le premier point d'une CoordinateSequence
      */
     static geom::Coordinate getFirstPoint(const geom::Curve* curve);
         
     /**
      * Obtient le dernier point d'une CoordinateSequence
      */
     static geom::Coordinate getLastPoint(const geom::Curve* curve);
         
     /**
      * Helper pour convertir clone() de Geometry vers Curve
      */
     static std::unique_ptr<geom::Curve> cloneToCurve(const geom::Curve* curve);
 };
 
 } // namespace overlayng
 } // namespace operation
 } // namespace geos