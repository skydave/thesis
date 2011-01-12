// set some flags


//#define WRITE_AVI
#define FLUID_3D








///@}
/// \mainpage
///
/// \section intro_sec �berblick
///
/// 
/// \htmlonly Dies ist die Dokumentation der Implementierung welche das Diplom Bruchbildung in starr-plastischen Gemengen begleitet.\endhtmlonly
/// \htmlonly Die Implementierung besteht aus:<br><br>\endhtmlonly
/// \htmlonly simulator - Einem Fluidsimulator mit verschiedenen Derivaten<br>\endhtmlonly
/// \htmlonly retile - Einem Algorithmus zum Retiling nachTurk'92<br>\endhtmlonly
/// \htmlonly mesher - Einem Algorithmus zur Oberfl�chenkonstruktion aus einer Partikelwolke nach Zhou2005<br>\endhtmlonly
/// \htmlonly cracksurface - Einem Algorithmus zur Berechnung von Oberfl�chenbr�chen in einem Dreiecksnetz nach IBEN2006<br>\endhtmlonly
/// \htmlonly CrackVisualizer - Einem 3dsmax-textur-plugin zur Visualisierung von Br�chen<br>\endhtmlonly
/// \htmlonly Viewer - Einem Program zum Betrachten der verschiedenen (Zwischen)Ergebnisse<br>\endhtmlonly
/// \htmlonly <br><br>\endhtmlonly
/// \htmlonly Diese Teile sind jeweils in eigene Projekte gegliedert und organisiert. Jedes Projekt hat seinen eigenen Quellkode in einem\endhtmlonly
/// \htmlonly Gleichnamigen Unterverzeichnis im source-verzeichnis. Quellkode, der �ber Projektgrenzen hinweg genutzt wird befindet sich\endhtmlonly
/// \htmlonly im Unterverzeichnis common. Dort finden sich beispielsweise die Mathematik-routinen und vieles mehr.\endhtmlonly
/// \htmlonly <br><br>\endhtmlonly
/// \htmlonly W�hrend diese Dokumentation einen sehr guten �berblick �ber die Strukturen und Teile der Arbeit erm�glicht, empfehle ich auch\endhtmlonly
/// \htmlonly den Blick in die Quellkodes, da diese auch in den Funktionsr�mpfen extensiv kommentiert sind. W�hrend der Arbeit wurde ein gro�es\endhtmlonly
/// \htmlonly Augenmerk auf eine sorgf�ltige und hilfreiche Kommentierung gelegt.\endhtmlonly
///
///