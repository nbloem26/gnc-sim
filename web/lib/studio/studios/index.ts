/**
 * Studio barrel — importing this module registers every built-in studio with
 * the registry (`@/lib/studio/registry`). The Studios area imports this once on
 * mount so `listStudios()` is populated.
 *
 * To add a studio: create `studios/<name>Studio.ts` exporting a `StudioDef`,
 * then import + register it here.
 */

import { registerStudio } from '../registry';
import { exampleStudio } from './exampleStudio';

// Sensing
import { radarStudio } from './radarStudio';
import { fusionStudio } from './fusionStudio';
// Guidance, control & estimation
import { guidanceStudio } from './guidanceStudio';
import { controlsStudio } from './controlsStudio';
import { filterStudio } from './filterStudio';
// Threats & environment
import { threatStudio } from './threatStudio';
import { environmentStudio } from './environmentStudio';
// Engagement & analysis
import { campaignStudio } from './campaignStudio';
import { uqStudio } from './uqStudio';

registerStudio(exampleStudio);
registerStudio(radarStudio);
registerStudio(fusionStudio);
registerStudio(guidanceStudio);
registerStudio(controlsStudio);
registerStudio(filterStudio);
registerStudio(threatStudio);
registerStudio(environmentStudio);
registerStudio(campaignStudio);
registerStudio(uqStudio);
