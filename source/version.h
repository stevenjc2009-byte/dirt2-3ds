/*---------------------------------------------------------------------------------
 * version.h -- the ONE place DIRT2_VERSION is defined.
 *
 * Every other place that needs to know "what version is this build" (the
 * pause-menu update check, a future about screen, a future crash report)
 * includes this header rather than carrying its own copy of the string. A
 * second hardcoded copy is how a project ships a build that reports the WRONG
 * version to itself -- see the sibling project mc's updater, which learned
 * this the hard way and keeps its own version in exactly one file
 * (source/app/updater_version.h) for the same reason.
 *
 * Bump this by hand immediately before cutting a release, so the binary that
 * gets tagged vX.Y.Z on GitHub is the one that reports X.Y.Z to its own
 * update check.
 *---------------------------------------------------------------------------------*/
#ifndef DIRT2_VERSION_H
#define DIRT2_VERSION_H

#define DIRT2_VERSION "1.0.2"

#endif /* DIRT2_VERSION_H */
