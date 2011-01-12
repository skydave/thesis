// set some flags


//#define WRITE_AVI
#define FLUID_3D








///@}
/// \mainpage
///
/// \section intro_sec Überblick
///
/// 
/// \htmlonly Dies ist die Dokumentation der Implementierung welche das Diplom Bruchbildung in starr-plastischen Gemengen begleitet.\endhtmlonly
/// \htmlonly Die Implementierung besteht aus:<br><br>\endhtmlonly
/// \htmlonly simulator - Einem Fluidsimulator mit verschiedenen Derivaten<br>\endhtmlonly
/// \htmlonly retile - Einem Algorithmus zum Retiling nachTurk'92<br>\endhtmlonly
/// \htmlonly mesher - Einem Algorithmus zur Oberflächenkonstruktion aus einer Partikelwolke nach Zhou2005<br>\endhtmlonly
/// \htmlonly cracksurface - Einem Algorithmus zur Berechnung von Oberflächenbrüchen in einem Dreiecksnetz nach IBEN2006<br>\endhtmlonly
/// \htmlonly CrackVisualizer - Einem 3dsmax-textur-plugin zur Visualisierung von Brüchen<br>\endhtmlonly
/// \htmlonly Viewer - Einem Program zum Betrachten der verschiedenen (Zwischen)Ergebnisse<br>\endhtmlonly
/// \htmlonly <br><br>\endhtmlonly
/// \htmlonly Diese Teile sind jeweils in eigene Projekte gegliedert und organisiert. Jedes Projekt hat seinen eigenen Quellkode in einem\endhtmlonly
/// \htmlonly Gleichnamigen Unterverzeichnis im source-verzeichnis. Quellkode, der über Projektgrenzen hinweg genutzt wird befindet sich\endhtmlonly
/// \htmlonly im Unterverzeichnis common. Dort finden sich beispielsweise die Mathematik-routinen und vieles mehr.\endhtmlonly
/// \htmlonly <br><br>\endhtmlonly
/// \htmlonly Während diese Dokumentation einen sehr guten Überblick über die Strukturen und Teile der Arbeit ermöglicht, empfehle ich auch\endhtmlonly
/// \htmlonly den Blick in die Quellkodes, da diese auch in den Funktionsrümpfen extensiv kommentiert sind. Während der Arbeit wurde ein großes\endhtmlonly
/// \htmlonly Augenmerk auf eine sorgfältige und hilfreiche Kommentierung gelegt.\endhtmlonly
///
///