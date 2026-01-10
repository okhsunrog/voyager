//! Interface Control Document - shared endpoint/topic definitions

use ergot::{endpoint, topic};

// Time service endpoints
endpoint!(SetTimeEndpoint, u64, (), "time/set");
endpoint!(GetTimeEndpoint, (), u64, "time/get");

// DFU endpoint
endpoint!(RebootToDfuEndpoint, (), (), "dfu/reboot");

// Topics for notifications
topic!(TimeUpdatedTopic, u64, "time/updated");
