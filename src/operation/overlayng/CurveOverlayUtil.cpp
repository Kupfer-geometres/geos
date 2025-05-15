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

 #include <geos/operation/overlayng/CurveOverlayUtil.h>
 #include <geos/geom/CircularString.h>
 #include <geos/geom/CompoundCurve.h>
 #include <geos/geom/CurvePolygon.h>
 #include <geos/geom/MultiCurve.h>
 #include <geos/geom/MultiSurface.h>
 #include <geos/geom/LineString.h>
 #include <geos/geom/Point.h>
 #include <geos/geom/MultiPoint.h>
 #include <geos/geom/GeometryCollection.h>
 #include <geos/geom/GeometryFactory.h>
 #include <geos/geom/CoordinateSequence.h>
 #include <geos/geom/Coordinate.h>
 #include <geos/geom/CircularArc.h>
 #include <geos/geom/Geometry.h>
 #include <geos/geom/Curve.h>
 #include <geos/operation/overlayng/OverlayNG.h>
 #include <geos/util/IllegalArgumentException.h>
 #include <geos/util/UnsupportedOperationException.h>
 #include <vector>
 #include <cmath>

 #include <iostream>  // Ajouter en haut du fichier si pas déjà présent

 
 // Définir M_PI si non disponible
 #ifndef M_PI
 #define M_PI 3.14159265358979323846
 #endif
 
 namespace geos {
 namespace operation {
 namespace overlayng {
 
 /* public static */
 bool 
 CurveOverlayUtil::hasCurvedComponents(const geom::Geometry* geom)
 {
     if (!geom) return false;
     
     geom::GeometryTypeId typeId = geom->getGeometryTypeId();
     
     // Check for direct curve types
     if (typeId == geom::GEOS_CIRCULARSTRING ||
         typeId == geom::GEOS_COMPOUNDCURVE ||
         typeId == geom::GEOS_CURVEPOLYGON ||
         typeId == geom::GEOS_MULTICURVE ||
         typeId == geom::GEOS_MULTISURFACE) {
         return true;
     }
     
     // Check collection types recursively
     if (typeId == geom::GEOS_GEOMETRYCOLLECTION) {
         for (std::size_t i = 0; i < geom->getNumGeometries(); i++) {
             if (hasCurvedComponents(geom->getGeometryN(i))) {
                 return true;
             }
         }
     }
     
     return false;
 }
 
 /* public static */
 std::unique_ptr<geom::Geometry>
 CurveOverlayUtil::overlayCurves(
     const geom::Geometry* geom0, 
     const geom::Geometry* geom1, 
     int opCode)
 {
     // Get the geometry factory from one of the input geometries
     const geom::GeometryFactory* factory = geom0->getFactory();
     
     switch (opCode) {
         case OverlayNG::INTERSECTION:
             return overlayCurvesIntersection(geom0, geom1, factory);
             
         case OverlayNG::UNION:
             return overlayCurvesUnion(geom0, geom1, factory);
             
         case OverlayNG::DIFFERENCE:
             return overlayCurvesDifference(geom0, geom1, factory);
             
         case OverlayNG::SYMDIFFERENCE:
             return overlayCurvesSymDifference(geom0, geom1, factory);
             
         default:
             throw util::IllegalArgumentException("Unknown overlay operation");
     }
 }
 
 /* private static */
 std::unique_ptr<geom::Geometry>
 CurveOverlayUtil::overlayCurvesIntersection(
     const geom::Geometry* geom0,
     const geom::Geometry* geom1,
     const geom::GeometryFactory* /*factory*/)
 {
     // Special case: CircularString vs LineString
     if (geom0->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING &&
         geom1->getGeometryTypeId() == geom::GEOS_LINESTRING) {
         const auto* cs = static_cast<const geom::CircularString*>(geom0);
         const auto* ls = static_cast<const geom::LineString*>(geom1);
         return intersectCircularStringLineString(cs, ls);
     }
     
     // Special case: LineString vs CircularString (swap order)
     if (geom0->getGeometryTypeId() == geom::GEOS_LINESTRING &&
         geom1->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
         const auto* ls = static_cast<const geom::LineString*>(geom0);
         const auto* cs = static_cast<const geom::CircularString*>(geom1);
         return intersectCircularStringLineString(cs, ls);
     }
     
     // Special case: CircularString vs CircularString
     if (geom0->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING &&
         geom1->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
         const auto* cs1 = static_cast<const geom::CircularString*>(geom0);
         const auto* cs2 = static_cast<const geom::CircularString*>(geom1);
         return intersectCircularStrings(cs1, cs2);
     }
     
     // For now, throw an exception for unsupported combinations
     throw util::UnsupportedOperationException(
         "Curved geometry intersection not yet implemented for this combination");
 }
 
 /* private static */
 std::unique_ptr<geom::Geometry>
 CurveOverlayUtil::intersectCircularStringLineString(
     const geom::CircularString* cs,
     const geom::LineString* ls)
 {
     std::vector<std::unique_ptr<geom::Point>> intersectionPoints;
     const geom::GeometryFactory* factory = cs->getFactory();
     
     // Get the coordinates from the circular string
     const geom::CoordinateSequence* csCoords = cs->getCoordinatesRO();
     
     // For each arc in the circular string (defined by 3 consecutive points)
     for (std::size_t i = 2; i < csCoords->size(); i += 2) {
         geom::Coordinate p1 = csCoords->getAt(i - 2);
         geom::Coordinate p2 = csCoords->getAt(i - 1);
         geom::Coordinate p3 = csCoords->getAt(i);
         
         geom::CircularArc arc(p1, p2, p3);
         
         // For each segment in the line string
         const geom::CoordinateSequence* lsCoords = ls->getCoordinatesRO();
         for (std::size_t j = 1; j < lsCoords->size(); j++) {
             geom::Coordinate segStart = lsCoords->getAt(j - 1);
             geom::Coordinate segEnd = lsCoords->getAt(j);
             
             // Find intersection between arc and line segment
             std::vector<geom::Coordinate> arcSegIntersections = 
                 intersectArcSegment(arc, segStart, segEnd);
             
             // Create Point geometries for each intersection
             for (const auto& coord : arcSegIntersections) {
                 intersectionPoints.push_back(
                     factory->createPoint(coord));
             }
         }
     }
     
     // Return the collection of intersection points
     if (intersectionPoints.empty()) {
         return factory->createEmptyGeometry();
     } else if (intersectionPoints.size() == 1) {
         return std::move(intersectionPoints[0]);
     } else {
         return factory->createMultiPoint(std::move(intersectionPoints));
     }
 }
 
 /* private static */
 std::vector<geom::Coordinate>
 CurveOverlayUtil::intersectArcSegment(
     const geom::CircularArc& arc,
     const geom::Coordinate& segStart,
     const geom::Coordinate& segEnd)
 {
     std::vector<geom::Coordinate> intersections;
     
     // Get arc center and radius using getCenter() which returns CoordinateXY
     auto centerXY = arc.getCenter();
     geom::Coordinate center(centerXY.x, centerXY.y);
     double radius = arc.getRadius();
     
     // Line segment as parametric equation: P = segStart + t * (segEnd - segStart)
     double dx = segEnd.x - segStart.x;
     double dy = segEnd.y - segStart.y;
     
     // Solve for intersection with circle
     double a = dx * dx + dy * dy;
     double b = 2 * (dx * (segStart.x - center.x) + dy * (segStart.y - center.y));
     double c = (segStart.x - center.x) * (segStart.x - center.x) + 
                (segStart.y - center.y) * (segStart.y - center.y) - 
                radius * radius;
     
     double discriminant = b * b - 4 * a * c;
     
     if (discriminant < 0) {
         // No intersection
         return intersections;
     }
     
     double t1 = (-b - std::sqrt(discriminant)) / (2 * a);
     double t2 = (-b + std::sqrt(discriminant)) / (2 * a);
     
     // Check if intersections are on the line segment (0 <= t <= 1)
     if (t1 >= 0 && t1 <= 1) {
         geom::Coordinate p1(segStart.x + t1 * dx, segStart.y + t1 * dy);
         
         // Check if the point is on the arc by comparing angles
         // Simplified check - proper implementation would check
         // if the point is between the arc's start and end angles
         double distToCenter = std::sqrt((p1.x - center.x) * (p1.x - center.x) + 
                                       (p1.y - center.y) * (p1.y - center.y));
         if (std::abs(distToCenter - radius) < 1e-9) {
             // TODO: Add proper angle check
             intersections.push_back(p1);
         }
     }
     
     if (t2 >= 0 && t2 <= 1 && std::abs(t2 - t1) > 1e-9) {
         geom::Coordinate p2(segStart.x + t2 * dx, segStart.y + t2 * dy);
         
         // Check if the point is on the arc by comparing angles
         // Simplified check - proper implementation would check
         // if the point is between the arc's start and end angles
         double distToCenter = std::sqrt((p2.x - center.x) * (p2.x - center.x) + 
                                       (p2.y - center.y) * (p2.y - center.y));
         if (std::abs(distToCenter - radius) < 1e-9) {
             // TODO: Add proper angle check
             intersections.push_back(p2);
         }
     }
     
     return intersections;
 }
 
 /* private static */
 std::unique_ptr<geom::Geometry>
 CurveOverlayUtil::intersectCircularStrings(
     const geom::CircularString* /*cs1*/,
     const geom::CircularString* /*cs2*/)
 {
     // TODO: Implement intersection between two circular strings
     // This is more complex as it involves arc-arc intersection
     
     throw util::UnsupportedOperationException(
         "CircularString-CircularString intersection not yet implemented");
 }
 
 /* private static */
 std::unique_ptr<geom::Geometry>
 CurveOverlayUtil::overlayCurvesUnion(
     const geom::Geometry* geom0,
     const geom::Geometry* geom1,
     const geom::GeometryFactory* factory)
 {
     // Special case: CircularString vs CircularString
     if (geom0->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING &&
         geom1->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
         const auto* cs1 = static_cast<const geom::CircularString*>(geom0);
         const auto* cs2 = static_cast<const geom::CircularString*>(geom1);
         return unionCircularStrings(cs1, cs2, factory);
     }
     
     // Special case: CurvePolygon vs CurvePolygon (circles)
     if (geom0->getGeometryTypeId() == geom::GEOS_CURVEPOLYGON &&
         geom1->getGeometryTypeId() == geom::GEOS_CURVEPOLYGON) {
         const auto* cp1 = static_cast<const geom::CurvePolygon*>(geom0);
         const auto* cp2 = static_cast<const geom::CurvePolygon*>(geom1);
         return unionCurvePolygons(cp1, cp2, factory);
     }
     
     // For now, just create a collection for other combinations
     std::vector<std::unique_ptr<geom::Geometry>> geoms;
     geoms.push_back(geom0->clone());
     geoms.push_back(geom1->clone());
     return factory->createGeometryCollection(std::move(geoms));
 }
 
 /* private static */
 std::unique_ptr<geom::Geometry>
 CurveOverlayUtil::overlayCurvesDifference(
     const geom::Geometry* /*geom0*/,
     const geom::Geometry* /*geom1*/,
     const geom::GeometryFactory* /*factory*/)
 {
     throw util::UnsupportedOperationException(
         "Curved geometry difference not yet implemented");
 }
 
 /* private static */
 std::unique_ptr<geom::Geometry>
 CurveOverlayUtil::overlayCurvesSymDifference(
     const geom::Geometry* /*geom0*/,
     const geom::Geometry* /*geom1*/,
     const geom::GeometryFactory* /*factory*/)
 {
     throw util::UnsupportedOperationException(
         "Curved geometry symmetric difference not yet implemented");
 }
 
 /* private static */
 std::unique_ptr<geom::Curve>
 CurveOverlayUtil::cloneToCurve(const geom::Curve* curve)
 {
     auto cloned = curve->clone();
     return std::unique_ptr<geom::Curve>(
         dynamic_cast<geom::Curve*>(cloned.release())
     );
 }
 
 /* private static */
 std::unique_ptr<geom::Geometry>
 CurveOverlayUtil::unionCircularStrings(
     const geom::CircularString* cs1,
     const geom::CircularString* cs2,
     const geom::GeometryFactory* factory)
 {
     // Pour les GIS, on garde toujours les points intermédiaires
     // donc on retourne directement un MultiCurve
     std::vector<std::unique_ptr<geom::Curve>> curvesForMultiCurve;
     
     curvesForMultiCurve.push_back(cloneToCurve(cs1));
     curvesForMultiCurve.push_back(cloneToCurve(cs2));
     
     return factory->createMultiCurve(std::move(curvesForMultiCurve));
 }
 

/* private static */
std::unique_ptr<geom::Geometry>
CurveOverlayUtil::unionCurvePolygons(
    const geom::CurvePolygon* cp1,
    const geom::CurvePolygon* cp2,
    const geom::GeometryFactory* factory)
{
    std::cout << "DEBUG: unionCurvePolygons - Début\n";
    
    // Étape 1: Extraire toutes les courbes des deux polygones
    std::vector<std::unique_ptr<geom::Curve>> allCurves;
    
    // Extraire les courbes du premier polygone
    auto curves1 = extractCurveSegments(cp1->getExteriorRing(), factory);
    std::cout << "DEBUG: Nombre de segments extraits du polygone 1: " << curves1.size() << "\n";
    for (auto& curve : curves1) {
        allCurves.push_back(std::move(curve));
    }
    
    // Extraire les courbes du second polygone
    auto curves2 = extractCurveSegments(cp2->getExteriorRing(), factory);
    std::cout << "DEBUG: Nombre de segments extraits du polygone 2: " << curves2.size() << "\n";
    for (auto& curve : curves2) {
        allCurves.push_back(std::move(curve));
    }
    
    std::cout << "DEBUG: Nombre total de courbes: " << allCurves.size() << "\n";
    
    // Étape 2: Trouver toutes les intersections
    std::vector<geom::Coordinate> intersectionPoints;
    for (size_t i = 0; i < allCurves.size(); i++) {
        for (size_t j = i + 1; j < allCurves.size(); j++) {
            std::cout << "DEBUG: Vérification intersection entre courbe " << i << " et " << j << "\n";
            // TODO: Gérer l'intersection entre différents types de courbes
            // Pour l'instant, supposons des CircularString
            if (allCurves[i]->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING &&
                allCurves[j]->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
                
                const auto* cs1 = static_cast<const geom::CircularString*>(allCurves[i].get());
                const auto* cs2 = static_cast<const geom::CircularString*>(allCurves[j].get());
                
                // Créer des arcs à partir des CircularString
                const auto* coords1 = cs1->getCoordinatesRO();
                const auto* coords2 = cs2->getCoordinatesRO();
                
                std::cout << "DEBUG: CircularString 1 a " << coords1->size() << " points\n";
                std::cout << "DEBUG: CircularString 2 a " << coords2->size() << " points\n";
                
                if (coords1->size() >= 3 && coords2->size() >= 3) {
                    geom::CircularArc arc1(
                        coords1->getAt(0),
                        coords1->getAt(1),
                        coords1->getAt(2)
                    );
                    
                    geom::CircularArc arc2(
                        coords2->getAt(0),
                        coords2->getAt(1),
                        coords2->getAt(2)
                    );
                    
                    std::cout << "DEBUG: Arc 1 - centre: (" << arc1.getCenter().x << ", " << arc1.getCenter().y 
                              << "), rayon: " << arc1.getRadius() << "\n";
                    std::cout << "DEBUG: Arc 2 - centre: (" << arc2.getCenter().x << ", " << arc2.getCenter().y 
                              << "), rayon: " << arc2.getRadius() << "\n";
                    
                    auto arcIntersections = intersectArcs(arc1, arc2);
                    std::cout << "DEBUG: Nombre d'intersections trouvées: " << arcIntersections.size() << "\n";
                    for (const auto& pt : arcIntersections) {
                        std::cout << "DEBUG: Point d'intersection: (" << pt.x << ", " << pt.y << ")\n";
                        intersectionPoints.push_back(pt);
                    }
                }
            }
        }
    }
    
    std::cout << "DEBUG: Nombre total de points d'intersection: " << intersectionPoints.size() << "\n";
    
    // Étape 3: Diviser les courbes aux points d'intersection
    std::vector<std::unique_ptr<geom::Curve>> splitCurves;
    for (auto& curve : allCurves) {
        std::vector<std::unique_ptr<geom::Curve>> curveSplits;
        curveSplits.push_back(cloneToCurve(curve.get()));
        
        // Pour chaque point d'intersection
        for (const auto& pt : intersectionPoints) {
            std::vector<std::unique_ptr<geom::Curve>> newSplits;
            
            for (auto& segment : curveSplits) {
                if (segment->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
                    const auto* cs = static_cast<const geom::CircularString*>(segment.get());
                    
                    // Vérifier si le point est sur cet arc
                    const auto* coords = cs->getCoordinatesRO();
                    if (coords->size() >= 3) {
                        geom::CircularArc arc(
                            coords->getAt(0),
                            coords->getAt(1),
                            coords->getAt(2)
                        );
                        
                        if (isPointOnArc(arc, pt)) {
                            std::cout << "DEBUG: Point de split trouvé sur l'arc\n";
                            // Diviser l'arc
                            auto splits = splitArc(cs, pt, factory);
                            std::cout << "DEBUG: Arc divisé en " << splits.size() << " parties\n";
                            for (auto& split : splits) {
                                newSplits.push_back(std::move(split));
                            }
                        } else {
                            newSplits.push_back(std::move(segment));
                        }
                    }
                } else {
                    // TODO: Gérer les autres types de courbes
                    newSplits.push_back(std::move(segment));
                }
            }
            
            curveSplits = std::move(newSplits);
        }
        
        for (auto& split : curveSplits) {
            splitCurves.push_back(std::move(split));
        }
    }
    
    std::cout << "DEBUG: Nombre de segments après division: " << splitCurves.size() << "\n";
    
    // Étape 4: Déterminer quelles parties sont à l'intérieur de l'union
    std::vector<std::unique_ptr<geom::Curve>> unionSegments;
    
    for (auto& segment : splitCurves) {
        // Prendre le point milieu du segment
        auto midPoint = getMidPoint(segment.get());
        std::cout << "DEBUG: Point milieu du segment: (" << midPoint.x << ", " << midPoint.y << ")\n";
        
        // Vérifier si ce point est dans au moins un des polygones originaux
        bool inPolygon1 = isPointInPolygon(midPoint, cp1);
        bool inPolygon2 = isPointInPolygon(midPoint, cp2);
        
        std::cout << "DEBUG: Dans polygone 1: " << inPolygon1 << ", Dans polygone 2: " << inPolygon2 << "\n";
        
        // Si le segment est dans au moins un polygone, il fait partie de l'union
        if (inPolygon1 || inPolygon2) {
            // Vérifier si ce n'est pas un segment intérieur partagé
            if (!(inPolygon1 && inPolygon2)) {
                // C'est un segment de bordure
                std::cout << "DEBUG: Segment de bordure ajouté à l'union\n";
                unionSegments.push_back(std::move(segment));
            } else {
                // Le segment est à l'intérieur des deux polygones
                // Il ne fait partie de la bordure que s'il correspond à une bordure originale
                bool onBoundary1 = isSegmentOnBoundary(segment.get(), cp1);
                bool onBoundary2 = isSegmentOnBoundary(segment.get(), cp2);
                
                std::cout << "DEBUG: Sur bordure 1: " << onBoundary1 << ", Sur bordure 2: " << onBoundary2 << "\n";
                
                if (onBoundary1 || onBoundary2) {
                    std::cout << "DEBUG: Segment de bordure partagée ajouté à l'union\n";
                    unionSegments.push_back(std::move(segment));
                }
            }
        }
    }
    
    std::cout << "DEBUG: Nombre de segments dans l'union: " << unionSegments.size() << "\n";
    
    // Étape 5: Construire le nouveau polygone union
    if (unionSegments.empty()) {
        std::cout << "DEBUG: Aucun segment dans l'union - retour géométrie vide\n";
        return factory->createEmptyGeometry();
    }
    
    // Organiser les segments en un anneau fermé
    auto ring = buildRingFromSegments(unionSegments, factory);
    
    if (ring) {
        std::cout << "DEBUG: Anneau construit avec succès\n";
        // Créer un CurvePolygon avec le nouvel anneau
        std::vector<std::unique_ptr<geom::Curve>> holes; // Pas de trous pour l'instant
        return factory->createCurvePolygon(std::move(ring), std::move(holes));
    } else {
        std::cout << "DEBUG: Impossible de construire un anneau - retour collection\n";
        // Si on ne peut pas construire un anneau, retourner les segments
        std::vector<std::unique_ptr<geom::Geometry>> parts;
        for (auto& seg : unionSegments) {
            parts.push_back(std::move(seg));
        }
        return factory->createGeometryCollection(std::move(parts));
    }
}
 
 /* private static */
 bool
 CurveOverlayUtil::isPointOnArc(
     const geom::CircularArc& arc,
     const geom::Coordinate& point,
     double epsilon)
 {
     // Vérifier d'abord si le point est sur le cercle
     auto center = arc.getCenter();
     double radius = arc.getRadius();
     
     double distanceToCenter = std::sqrt(
         (point.x - center.x) * (point.x - center.x) +
         (point.y - center.y) * (point.y - center.y)
     );
     
     if (std::abs(distanceToCenter - radius) > epsilon) {
         return false;
     }
     
     // Calculer les angles des points de l'arc
     auto anglePoint = std::atan2(point.y - center.y, point.x - center.x);
     auto angleP0 = std::atan2(arc.p0.y - center.y, arc.p0.x - center.x);
     auto angleP1 = std::atan2(arc.p1.y - center.y, arc.p1.x - center.x);
     auto angleP2 = std::atan2(arc.p2.y - center.y, arc.p2.x - center.x);
     
     // Normaliser les angles entre 0 et 2π
     if (anglePoint < 0) anglePoint += 2 * M_PI;
     if (angleP0 < 0) angleP0 += 2 * M_PI;
     if (angleP1 < 0) angleP1 += 2 * M_PI;
     if (angleP2 < 0) angleP2 += 2 * M_PI;
     
     // Déterminer si l'arc est dans le sens horaire ou anti-horaire
     bool isClockwise = ((angleP1 > angleP0 && angleP1 < angleP2) || 
                        (angleP0 > angleP2 && (angleP1 > angleP0 || angleP1 < angleP2)));
     
     if (!isClockwise) {
         // Anti-horaire
         if (angleP0 < angleP2) {
             return anglePoint >= angleP0 && anglePoint <= angleP2;
         } else {
             return anglePoint >= angleP0 || anglePoint <= angleP2;
         }
     } else {
         // Horaire
         if (angleP0 > angleP2) {
             return anglePoint >= angleP2 && anglePoint <= angleP0;
         } else {
             return anglePoint <= angleP0 || anglePoint >= angleP2;
         }
     }
 }
 
 /* private static */
 std::vector<std::unique_ptr<geom::CircularString>>
 CurveOverlayUtil::splitArc(
     const geom::CircularString* arc,
     const geom::Coordinate& splitPoint,
     const geom::GeometryFactory* factory)
 {
     std::vector<std::unique_ptr<geom::CircularString>> result;
     
     const auto* coords = arc->getCoordinatesRO();
     if (coords->size() < 3) {
         result.push_back(arc->clone());
         return result;
     }
     
     // Créer un arc à partir des coordonnées
     geom::CircularArc arcGeom(
         coords->getAt(0),
         coords->getAt(1),
         coords->getAt(2)
     );
     
     // Vérifier que le point est sur l'arc
     if (!isPointOnArc(arcGeom, splitPoint)) {
         result.push_back(arc->clone());
         return result;
     }
     
     // Si le point est une extrémité, pas de split
     if (splitPoint.equals2D(coords->getAt(0)) || splitPoint.equals2D(coords->getAt(2))) {
         result.push_back(arc->clone());
         return result;
     }
     
     // Calculer les angles
     auto center = arcGeom.getCenter();
     auto radius = arcGeom.getRadius();
     
     auto angleP0 = std::atan2(coords->getAt(0).y - center.y, coords->getAt(0).x - center.x);
     auto angleP2 = std::atan2(coords->getAt(2).y - center.y, coords->getAt(2).x - center.x);
     auto angleSplit = std::atan2(splitPoint.y - center.y, splitPoint.x - center.x);
     
     // Calculer des points intermédiaires pour les deux nouveaux arcs
     // Premier arc : de p0 à splitPoint
     auto midAngle1 = (angleP0 + angleSplit) / 2;
     geom::Coordinate midPoint1(
         center.x + radius * std::cos(midAngle1),
         center.y + radius * std::sin(midAngle1)
     );
     
     // Second arc : de splitPoint à p2
     auto midAngle2 = (angleSplit + angleP2) / 2;
     geom::Coordinate midPoint2(
         center.x + radius * std::cos(midAngle2),
         center.y + radius * std::sin(midAngle2)
     );
     
     // Créer les deux nouveaux arcs
     std::unique_ptr<geom::CoordinateSequence> coords1(new geom::CoordinateSequence(3, 2));
     coords1->setAt(coords->getAt(0), 0);
     coords1->setAt(midPoint1, 1);
     coords1->setAt(splitPoint, 2);
     
     std::unique_ptr<geom::CoordinateSequence> coords2(new geom::CoordinateSequence(3, 2));
     coords2->setAt(splitPoint, 0);
     coords2->setAt(midPoint2, 1);
     coords2->setAt(coords->getAt(2), 2);
     
     result.push_back(factory->createCircularString(std::move(coords1)));
     result.push_back(factory->createCircularString(std::move(coords2)));
     
     return result;
 }
 
 /* private static */
 std::vector<geom::Coordinate>
 CurveOverlayUtil::intersectArcs(
     const geom::CircularArc& arc1,
     const geom::CircularArc& arc2)
 {
     std::vector<geom::Coordinate> intersections;
     
     // Distance entre les centres
     auto center1 = arc1.getCenter();
     auto center2 = arc2.getCenter();
     double d = center1.distance(center2);
     
     double r1 = arc1.getRadius();
     double r2 = arc2.getRadius();
     
     // Si les centres sont identiques
     if (d < 1e-9) {
         // Les arcs sont sur le même cercle
         if (std::abs(r1 - r2) < 1e-9) {
             // Vérifier s'ils se chevauchent
             // Vérifier les extrémités - créer des Coordinate à partir de CoordinateXY
             geom::Coordinate p0_1(arc1.p0.x, arc1.p0.y);
             geom::Coordinate p2_1(arc1.p2.x, arc1.p2.y);
             geom::Coordinate p0_2(arc2.p0.x, arc2.p0.y);
             geom::Coordinate p2_2(arc2.p2.x, arc2.p2.y);
             
             if (isPointOnArc(arc2, p0_1)) {
                 intersections.push_back(p0_1);
             }
             if (isPointOnArc(arc2, p2_1)) {
                 intersections.push_back(p2_1);
             }
             if (isPointOnArc(arc1, p0_2)) {
                 // Éviter les doublons
                 bool isDuplicate = false;
                 for (const auto& pt : intersections) {
                     if (pt.equals2D(p0_2)) {
                         isDuplicate = true;
                         break;
                     }
                 }
                 if (!isDuplicate) {
                     intersections.push_back(p0_2);
                 }
             }
             if (isPointOnArc(arc1, p2_2)) {
                 // Éviter les doublons
                 bool isDuplicate = false;
                 for (const auto& pt : intersections) {
                     if (pt.equals2D(p2_2)) {
                         isDuplicate = true;
                         break;
                     }
                 }
                 if (!isDuplicate) {
                     intersections.push_back(p2_2);
                 }
             }
         }
         return intersections;
     }
     
     // Vérifier si les cercles s'intersectent
     if (d > r1 + r2 + 1e-9) return intersections; // Trop éloignés
     if (d < std::abs(r1 - r2) - 1e-9) return intersections; // Un dans l'autre
     
     // Calculer les points d'intersection des cercles complets
     double a = (r1*r1 - r2*r2 + d*d) / (2 * d);
     double h_squared = r1*r1 - a*a;
     
     if (h_squared < 0) return intersections;
     
     double h = std::sqrt(h_squared);
     
     // Point médian
     double px = center1.x + a * (center2.x - center1.x) / d;
     double py = center1.y + a * (center2.y - center1.y) / d;
     
     // Direction perpendiculaire
     double perp_x = -(center2.y - center1.y) / d;
     double perp_y = (center2.x - center1.x) / d;
     
     // Les deux points d'intersection possibles
     geom::Coordinate p1(px + h * perp_x, py + h * perp_y);
     geom::Coordinate p2(px - h * perp_x, py - h * perp_y);
     
     // Vérifier si ces points sont sur les deux arcs
     if (isPointOnArc(arc1, p1) && isPointOnArc(arc2, p1)) {
         intersections.push_back(p1);
     }
     
     if (isPointOnArc(arc1, p2) && isPointOnArc(arc2, p2)) {
         intersections.push_back(p2);
     }
     
     return intersections;
 }
 
 /* private static */
 std::vector<std::unique_ptr<geom::Curve>>
 CurveOverlayUtil::extractCurveSegments(
     const geom::Curve* curve,
     const geom::GeometryFactory* factory)
 {
     std::vector<std::unique_ptr<geom::Curve>> segments;
     
     if (curve->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
         // Une CircularString est déjà un segment
         const auto* cs = static_cast<const geom::CircularString*>(curve);
         segments.push_back(cloneToCurve(cs));
     }
     else if (curve->getGeometryTypeId() == geom::GEOS_COMPOUNDCURVE) {
         // Une CompoundCurve contient plusieurs segments
         const auto* cc = static_cast<const geom::CompoundCurve*>(curve);
         for (std::size_t i = 0; i < cc->getNumCurves(); i++) {
             segments.push_back(cloneToCurve(cc->getCurveN(i)));
         }
     }
     else if (curve->getGeometryTypeId() == geom::GEOS_LINESTRING) {
         // Une LineString simple
         const auto* ls = static_cast<const geom::LineString*>(curve);
         segments.push_back(cloneToCurve(ls));
     }
     
     return segments;
 }
 
 /* private static */
 geom::Coordinate
 CurveOverlayUtil::getMidPoint(const geom::Curve* curve)
 {
     if (curve->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
         const auto* cs = static_cast<const geom::CircularString*>(curve);
         const auto* coords = cs->getCoordinatesRO();
         
         if (coords->size() >= 3) {
             // Pour un arc, utiliser le point du milieu (déjà fourni)
             return coords->getAt(1);
         }
     }
     else if (curve->getGeometryTypeId() == geom::GEOS_LINESTRING) {
         const auto* ls = static_cast<const geom::LineString*>(curve);
         const auto* coords = ls->getCoordinatesRO();
         
         if (coords->size() >= 2) {
             // Pour un segment, calculer le milieu
             auto p1 = coords->getAt(0);
             auto p2 = coords->getAt(coords->size() - 1);
             return geom::Coordinate(
                 (p1.x + p2.x) / 2,
                 (p1.y + p2.y) / 2
             );
         }
     }
     
     // Par défaut, utiliser le premier point
     if (curve->getNumPoints() > 0) {
         // getCoordinateN n'existe pas sur Curve, il faut caster
         if (curve->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
             const auto* cs = static_cast<const geom::CircularString*>(curve);
             return cs->getCoordinateN(0);
         } else if (curve->getGeometryTypeId() == geom::GEOS_LINESTRING) {
             const auto* ls = static_cast<const geom::LineString*>(curve);
             return ls->getCoordinateN(0);
         }
     }
     
     return geom::Coordinate();
 }
 
/* private static */
bool
CurveOverlayUtil::isPointInPolygon(
    const geom::Coordinate& point,
    const geom::CurvePolygon* polygon)
{
    // Créer un point geometry
    auto pt = polygon->getFactory()->createPoint(point);
    
    // Utiliser la méthode covers() ou contains() si disponible
    // Sinon, implémenter un test ray-casting simple
    
    // Pour l'instant, on va simplifier en testant si le point est
    // à l'intérieur du cercle défini par le CurvePolygon
    const auto* exteriorRing = polygon->getExteriorRing();
    
    if (exteriorRing->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
        const auto* cs = static_cast<const geom::CircularString*>(exteriorRing);
        const auto* coords = cs->getCoordinatesRO();
        
        // Si c'est un cercle complet (5 points avec premier = dernier)
        if (coords->size() == 5) {
            // Calculer le centre et le rayon
            geom::CircularArc arc(
                coords->getAt(0),
                coords->getAt(1),
                coords->getAt(2)
            );
            
            auto center = arc.getCenter();
            double radius = arc.getRadius();
            
            // Vérifier si le point est à l'intérieur du cercle
            double dist = std::sqrt(
                (point.x - center.x) * (point.x - center.x) +
                (point.y - center.y) * (point.y - center.y)
            );
            
            return dist <= radius;
        }
    }
    
    // Pour les autres cas, retourner false par défaut
    return false;
}
 
 /* private static */
 bool
 CurveOverlayUtil::isSegmentOnBoundary(
     const geom::Curve* segment,
     const geom::CurvePolygon* polygon)
 {
     // Vérifier si le segment correspond à une partie de la bordure
     const auto* exteriorRing = polygon->getExteriorRing();
     
     // Extraire les segments de la bordure
     auto boundarySegments = extractCurveSegments(exteriorRing, polygon->getFactory());
     
     // Comparer avec chaque segment de la bordure
     for (const auto& boundarySeg : boundarySegments) {
         // Vérifier si les segments sont identiques
         if (segmentsAreEqual(segment, boundarySeg.get())) {
             return true;
         }
     }
     
     return false;
 }
 
 /* private static */
 bool
 CurveOverlayUtil::segmentsAreEqual(
     const geom::Curve* seg1,
     const geom::Curve* seg2)
 {
     // Comparer les types
     if (seg1->getGeometryTypeId() != seg2->getGeometryTypeId()) {
         return false;
     }
     
     // Comparer par le nombre de points
     if (seg1->getNumPoints() != seg2->getNumPoints()) {
         return false;
     }
     
     // Comparer les points selon le type
     if (seg1->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
         const auto* cs1 = static_cast<const geom::CircularString*>(seg1);
         const auto* cs2 = static_cast<const geom::CircularString*>(seg2);
         
         const auto* coords1 = cs1->getCoordinatesRO();
         const auto* coords2 = cs2->getCoordinatesRO();
         
         for (size_t i = 0; i < coords1->size(); i++) {
             if (!coords1->getAt(i).equals2D(coords2->getAt(i))) {
                 return false;
             }
         }
         return true;
     }
     else if (seg1->getGeometryTypeId() == geom::GEOS_LINESTRING) {
         const auto* ls1 = static_cast<const geom::LineString*>(seg1);
         const auto* ls2 = static_cast<const geom::LineString*>(seg2);
         
         const auto* coords1 = ls1->getCoordinatesRO();
         const auto* coords2 = ls2->getCoordinatesRO();
         
         for (size_t i = 0; i < coords1->size(); i++) {
             if (!coords1->getAt(i).equals2D(coords2->getAt(i))) {
                 return false;
             }
         }
         return true;
     }
     
     return false;
 }
 
 /* private static */
std::unique_ptr<geom::Curve>
CurveOverlayUtil::buildRingFromSegments(
    std::vector<std::unique_ptr<geom::Curve>>& segments,
    const geom::GeometryFactory* factory)
{
    if (segments.empty()) {
        return nullptr;
    }
    
    // Organiser les segments en ordre
    std::vector<std::unique_ptr<geom::Curve>> orderedSegments;
    orderedSegments.push_back(std::move(segments[0]));
    segments.erase(segments.begin());
    
    while (!segments.empty()) {
        bool found = false;
        
        // Chercher le segment suivant - obtenir le dernier point
        geom::Coordinate lastPoint;
        const auto& lastSeg = orderedSegments.back();
        
        if (lastSeg->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
            const auto* cs = static_cast<const geom::CircularString*>(lastSeg.get());
            const auto* coords = cs->getCoordinatesRO();
            lastPoint = coords->getAt(coords->size() - 1);
        }
        else if (lastSeg->getGeometryTypeId() == geom::GEOS_LINESTRING) {
            const auto* ls = static_cast<const geom::LineString*>(lastSeg.get());
            const auto* coords = ls->getCoordinatesRO();
            lastPoint = coords->getAt(coords->size() - 1);
        }
        
        for (auto it = segments.begin(); it != segments.end(); ++it) {
            geom::Coordinate firstPoint;
            
            if ((*it)->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
                const auto* cs = static_cast<const geom::CircularString*>((*it).get());
                const auto* coords = cs->getCoordinatesRO();
                firstPoint = coords->getAt(0);
            }
            else if ((*it)->getGeometryTypeId() == geom::GEOS_LINESTRING) {
                const auto* ls = static_cast<const geom::LineString*>((*it).get());
                const auto* coords = ls->getCoordinatesRO();
                firstPoint = coords->getAt(0);
            }
            
            if (lastPoint.equals2D(firstPoint)) {
                // Ce segment suit le précédent
                orderedSegments.push_back(std::move(*it));
                segments.erase(it);
                found = true;
                break;
            }
        }
        
        if (!found) {
            // Impossible de former un anneau fermé
            return nullptr;
        }
    }
    
    // Créer un CompoundCurve à partir des segments ordonnés
    std::vector<std::unique_ptr<geom::SimpleCurve>> curves;
    for (auto& seg : orderedSegments) {
        if (seg->getGeometryTypeId() == geom::GEOS_CIRCULARSTRING) {
            const auto* cs = static_cast<const geom::CircularString*>(seg.get());
            curves.push_back(std::unique_ptr<geom::SimpleCurve>(
                dynamic_cast<geom::SimpleCurve*>(cs->clone().release())
            ));
        }
        else if (seg->getGeometryTypeId() == geom::GEOS_LINESTRING) {
            const auto* ls = static_cast<const geom::LineString*>(seg.get());
            curves.push_back(std::unique_ptr<geom::SimpleCurve>(
                dynamic_cast<geom::SimpleCurve*>(ls->clone().release())
            ));
        }
    }
    
    if (!curves.empty()) {
        return factory->createCompoundCurve(std::move(curves));
    }
    
    return nullptr;
}
 
 } // namespace overlayng
 } // namespace operation
 } // namespace geos