/**
 * Studio barrel — importing this module registers every built-in studio with
 * the registry (`@/lib/studio/registry`). The Studios area imports this once on
 * mount so `listStudios()` is populated.
 *
 * To add a studio: create `studios/<name>.ts` exporting a `StudioDef`, then
 * register it here.
 */

import { registerStudio } from '../registry';
import { exampleStudio } from './exampleStudio';

registerStudio(exampleStudio);
