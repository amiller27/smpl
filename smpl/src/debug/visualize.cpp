#include <smpl/debug/visualize.h>

#include <fstream>
#include <mutex>
#include <unordered_map>

namespace sbpl {
namespace viz {

namespace impl { // debug visualization level manager implementation

// void* in this context is a pointer to some implement-defined struct that
// manages the state of a named visualization

struct DebugViz {
    levels::Level level = levels::Info;
};

static std::unordered_map<std::string, DebugViz> g_visualizations;
static bool g_initialized = false;

const char *to_cstring(levels::Level level) {
    switch (level) {
    case levels::Debug:
        return "DEBUG";
    case levels::Info:
        return "INFO";
    case levels::Warn:
        return "WARN";
    case levels::Error:
        return "ERROR";
    case levels::Fatal:
        return "FATAL";
    default:
        return "";
    }
}

bool ParseVisualizationConfigLine(
    const std::string& line,
    std::vector<std::string>& split,
    levels::Level& level)
{
    split.clear();

    size_t last = 0;
    size_t next;
    bool done = false;
    while (!done) {
        next = line.find_first_of(".=", last);
        if (next == std::string::npos) { // no remaining . or =
            split.clear();
            done = true;
        } else if (line[next] == '.') {
            if (last == next) {
                // should be a word between successive .'s or ='s
                split.clear();
                done = true;
            } else {
                split.push_back(line.substr(last, next - last));
                last = next + 1;
            }
        } else if (line[next] == '=') {
            if (last == next) {
                split.clear();
            } else {
                split.push_back(line.substr(last, next - last));
                // parse the right hand side
                std::string rhs(line.substr(next + 1));
                if (rhs == "DEBUG") {
                    level = levels::Debug;
                } else if (rhs == "INFO") {
                    level = levels::Info;
                } else if (rhs == "WARN") {
                    level = levels::Warn;
                } else if (rhs == "ERROR") {
                    level = levels::Error;
                } else if (rhs == "FATAL") {
                    level = levels::Fatal;
                } else {
                    split.clear();
                }
            }
            done = true;
        }
    }

    return !split.empty();
}

void Initialize()
{
    if (g_initialized) {
        return;
    }

    const char *config_path = getenv("SMPL_VISUALIZE_CONFIG_FILE");
    if (config_path) {
        std::ifstream f(config_path);
        if (f.is_open()) {
            std::string line;
            while (f.good()) {
                std::getline(f, line);

                std::vector<std::string> split;
                levels::Level level;
                if (ParseVisualizationConfigLine(line, split, level)) {
                    std::string name = split.front();
                    for (size_t i = 1; i < split.size(); ++i) {
                        name = name + "." + split[i];
                    }
                    DebugViz viz;
                    viz.level = level;
                    g_visualizations[name] = viz;
                }
            }
        }
    }

    g_initialized = true;
}

void* GetHandle(const std::string& name)
{
    Initialize();
    return &g_visualizations[name];
}

bool IsEnabledFor(void* handle, levels::Level level)
{
    Initialize();
    return static_cast<DebugViz*>(handle)->level <= level;
}

void GetVisualizations(std::unordered_map<std::string, levels::Level>& visualizations)
{
    Initialize();
    visualizations.clear();
    for (auto& entry : g_visualizations) {
        visualizations.insert(std::make_pair(entry.first, entry.second.level));
    }
}

bool SetVisualizationLevel(const std::string& name, levels::Level level)
{
    Initialize();
    auto& vis = g_visualizations[name];
    if (vis.level != level) {
        vis.level = level;
        return true;
    }
    return false;
}

} // namespace impl

// global singly-linked list of locations, for batch updates when debug
// visualization levels have changed
static VizLocation* g_loc_head = nullptr;
static VizLocation* g_loc_tail = nullptr;

// thread-safe initialization of debug visualization locations
static std::mutex g_locations_mutex;

// global visualizer
static VisualizerBase* g_visualizer;

// guards calls to global visualizer's visualize() method
static std::mutex g_viz_mutex;

void CheckLocationEnabledNoLock(VizLocation* loc)
{
    loc->enabled = impl::IsEnabledFor(loc->handle, loc->level);
}

void InitializeVizLocation(
    VizLocation* loc,
    const std::string& name,
    levels::Level level)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);

    if (loc->initialized) {
        return;
    }

    loc->handle = impl::GetHandle(name);
    loc->level = level;

    if (!g_loc_head) {
        // list is empty
        assert(!g_loc_tail);
        g_loc_head = loc;
        g_loc_tail = loc;
    } else {
        g_loc_tail->next = loc;
        g_loc_tail = loc;
    }

    CheckLocationEnabledNoLock(loc);

    loc->initialized = true;
}

void NotifyLevelsChanged()
{
    for (VizLocation *loc = g_loc_head; loc; loc = loc->next) {
        CheckLocationEnabledNoLock(loc);
    }
}

// Manage the global visualizer. NOT thread-safe!
void set_visualizer(VisualizerBase* visualizer)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    g_visualizer = visualizer;
}

void unset_visualizer()
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    g_visualizer = nullptr;
}

VisualizerBase* visualizer()
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    return g_visualizer;
}

void get_visualizations(std::unordered_map<std::string, levels::Level>& visualizations)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);
    impl::GetVisualizations(visualizations);
}

bool set_visualization_level(const std::string& name, levels::Level level)
{
    std::unique_lock<std::mutex> lock(g_locations_mutex);
    bool changed = impl::SetVisualizationLevel(name, level);
    if (changed) {
        NotifyLevelsChanged();
    }
    return changed;
}

void visualize(
    levels::Level level,
    const visualization_msgs::MarkerArray& markers)
{
    std::unique_lock<std::mutex> lock(g_viz_mutex);
    if (!g_visualizer) {
        return;
    }

    g_visualizer->visualize(level, markers);
}

} // namespace viz
} // namespace sbpl
