#include "MapRenderer.h"
#include <algorithm>
#if defined(_WIN32)
#include <windows.h>
#endif
#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
namespace autotaxi {
MapRenderer::~MapRenderer() {
#if defined(_WIN32)
    if (!wglGetCurrentContext())
        return;
#endif
    if (list_)
        glDeleteLists(list_, 1);
}
bool MapRenderer::draw(const PanelFrame &frame, std::size_t revision, double left, double top) {
    if (frame.staticMapBegin == frame.staticMapEnd)
        return true;
    if (!list_) {
        list_ = glGenLists(1);
        if (!list_)
            return false;
    }
    if (!compileCount_ || revision_ != revision) {
        // Store window-local geometry so moving the window does not rebuild the airport.
        glNewList(list_, GL_COMPILE);
        for (std::size_t i = frame.staticMapBegin; i < frame.staticMapEnd;) {
            // The cached map range also contains labels and marker rectangles.
            // Only line geometry belongs in this display list; the remaining
            // commands are replayed by TaxiUI so text is rendered with the
            // font atlas instead of being interpreted as a zero-ended line.
            if (frame.commands[i].kind != DrawKind::Line) {
                ++i;
                continue;
            }
            const auto &command = frame.commands[i];
            const auto end = std::min(lineBatchEnd(frame.commands, i), frame.staticMapEnd);
            glLineWidth(static_cast<float>(command.width));
            glBegin(GL_LINES);
            for (; i < end; ++i) {
                const auto &line = frame.commands[i];
                glColor3f(line.color.r, line.color.g, line.color.b);
                glVertex2d(line.box.x, -line.box.y);
                glVertex2d(line.x2, -line.y2);
            }
            glEnd();
        }
        glEndList();
        revision_ = revision;
        ++compileCount_;
    }
    GLint matrixMode = GL_MODELVIEW;
    glGetIntegerv(GL_MATRIX_MODE, &matrixMode);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(left, top, 0);
    glPushAttrib(GL_CURRENT_BIT | GL_LINE_BIT);
    glCallList(list_);
    glPopAttrib();
    glPopMatrix();
    glMatrixMode(static_cast<GLenum>(matrixMode));
    return true;
}
} // namespace autotaxi
