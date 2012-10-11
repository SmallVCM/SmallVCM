/*
 * Copyright (C) 2012, Tomas Davidovic (http://www.davidovic.cz)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * (The above is MIT License: http://en.wikipedia.org/wiki/MIT_License)
 */

#ifndef __HTML_WRITER_HXX__
#define __HTML_WRITER_HXX__

// silence vsnprintf secure warning in MSVS
#pragma warning(push)
#pragma warning(disable : 4996)

#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdarg>

class HtmlWriter
{
public:

    enum BorderColor
    {
        kNone,
        kRed,
        kGreen
    };

public:

    HtmlWriter(const std::string& aFileName) :
        mFileName(aFileName),
        mHtml(mFileName.c_str())
    {
        // Most browsers will cap number of columns to real value,
        // so having something significantly larger works ok
        mAlgorithmCount = 100;
        mThumbnailSize  = 128;
    }

    ~HtmlWriter()
    {
        mHtml.flush();
        mHtml.close();
    }

    void Close()
    {
        mHtml << "</body>" << std::endl;
        mHtml << "</html>" << std::endl;
    }

    /*
     * The Javascript plugin is heavily modified version of plugin example:
     * http://www.htmldrive.net/items/show/838/jQuery-different-Photo-comparing-Effect
     *
     * Original copyright reads:
     * // Queness Before & After jQuery Plugin
     * // Created by Kevin Liew from Queness.com
     * // Permission is given to use this plugin in whatever way you want :)
     */
    void WriteHeader()
    {
        mHtml << "<!--"  << std::endl;
        mHtml << "* The Javascript plugin is heavily modified version of plugin example:"  << std::endl;
        mHtml << "* http://www.htmldrive.net/items/show/838/jQuery-different-Photo-comparing-Effect"  << std::endl;
        mHtml << "*"  << std::endl;
        mHtml << "* Original copyright reads:"  << std::endl;
        mHtml << "* // Queness Before & After jQuery Plugin"  << std::endl;
        mHtml << "* // Created by Kevin Liew from Queness.com"  << std::endl;
        mHtml << "* // Permission is given to use this plugin in whatever way you want :)"  << std::endl;
        mHtml << "-->" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "<!DOCTYPE html PUBLIC "
            "\"-//W3C//DTD XHTML 1.0 Strict//EN\""
            " \"http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd\">"
            << std::endl;
        mHtml << "<html xmlns=\"http://www.w3.org/1999/xhtml\">" << std::endl;
        mHtml << "<head>" << std::endl;
        mHtml << "<title>Comparison of GI algorithms with Vertex Connection Merging</title>" << std::endl;
        mHtml << "<meta http-equiv=\"Content-Language\" content=\"English\" />" << std::endl;
        mHtml << "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=UTF-8\" />" << std::endl;
        mHtml << "<script type=\"text/javascript\" src=\"http://ajax.googleapis.com/ajax/libs/jquery/1.4.4/jquery.min.js\"></script>" << std::endl;
        mHtml << "<script type=\"text/javascript\">" << std::endl;
        mHtml << "<!--" << std::endl;
        mHtml << "    (function($){" << std::endl;
        mHtml << "    $.fn.extend({" << std::endl;
        mHtml << "        //plugin name - qbeforeafter" << std::endl;
        mHtml << "        qbeforeafter: function(options) {" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "            return this.each(function() {" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                var i = $(this);" << std::endl;
        mHtml << "                var img_layer_tl = i.children('img:eq(0)').attr('src');" << std::endl;
        mHtml << "                var img_layer_tr = i.children('img:eq(1)').attr('src');" << std::endl;
        mHtml << "                var img_layer_bl = i.children('img:eq(2)').attr('src');" << std::endl;
        mHtml << "                var img_layer_br = i.children('img:eq(3)').attr('src');" << std::endl;
        mHtml << "                var img_cap_tl = i.children('img:eq(0)').attr('alt');" << std::endl;
        mHtml << "                var img_cap_tr = i.children('img:eq(1)').attr('alt');" << std::endl;
        mHtml << "                var img_cap_bl = i.children('img:eq(2)').attr('alt');" << std::endl;
        mHtml << "                var img_cap_br = i.children('img:eq(3)').attr('alt');" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                var img_style_tl = i.children('img:eq(0)').attr('style');" << std::endl;
        mHtml << "                var img_style_tr = i.children('img:eq(1)').attr('style');" << std::endl;
        mHtml << "                var img_style_bl = i.children('img:eq(2)').attr('style');" << std::endl;
        mHtml << "                var img_style_br = i.children('img:eq(3)').attr('style');" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                var width = i.children('img:eq(0)').width();" << std::endl;
        mHtml << "                var height = i.children('img:eq(0)').height();" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('img').hide();" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.css({'overflow': 'hidden', 'position': 'relative'});" << std::endl;
        mHtml << "                i.append('<div class=\"ba-layer_tl\"></div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-layer_tr\"></div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-layer_bl\"></div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-layer_br\"></div>');" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.append('<div class=\"ba-border_tl\" style=\"' + img_style_tl + '\"></div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-border_tr\" style=\"' + img_style_tr + '\"></div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-border_bl\" style=\"' + img_style_bl + '\"></div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-border_br\" style=\"' + img_style_br + '\"></div>');" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.append('<div class=\"ba-caption_tl\">' + img_cap_tl + '</div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-caption_tr\">' + img_cap_tr + '</div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-caption_bl\">' + img_cap_bl + '</div>');" << std::endl;
        mHtml << "                i.append('<div class=\"ba-caption_br\">' + img_cap_br + '</div>');" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-layer_tl, .ba-layer_tr, .ba-layer_bl, .ba-layer_br').width(width);" << std::endl;
        mHtml << "                i.children('.ba-layer_tl, .ba-layer_tr, .ba-layer_bl, .ba-layer_br').height(height);" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-layer_tl').css('backgroundImage','url(' + img_layer_tl + ')');" << std::endl;
        mHtml << "                i.children('.ba-layer_tr').css('backgroundImage','url(' + img_layer_tr + ')');" << std::endl;
        mHtml << "                i.children('.ba-layer_bl').css('backgroundImage','url(' + img_layer_bl + ')');" << std::endl;
        mHtml << "                i.children('.ba-layer_br').css('backgroundImage','url(' + img_layer_br + ')');" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-layer_tl').width(width * 0.5);" << std::endl;
        mHtml << "                i.children('.ba-layer_tl').height(height * 0.5);" << std::endl;
        mHtml << "                i.children('.ba-layer_tr').height(height * 0.5);" << std::endl;
        mHtml << "                i.children('.ba-layer_bl').width(width * 0.5);" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-caption_tl').show();" << std::endl;
        mHtml << "                i.children('.ba-caption_tr').show();" << std::endl;
        mHtml << "                i.children('.ba-caption_bl').show();" << std::endl;
        mHtml << "                i.children('.ba-caption_br').show();" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-caption_tl').css({ bottom: height * 0.5, right: width * 0.5 });" << std::endl;
        mHtml << "                i.children('.ba-caption_tr').css({ bottom: height * 0.5, left:  width * 0.5 });" << std::endl;
        mHtml << "                i.children('.ba-caption_bl').css({ top:    height * 0.5, right: width * 0.5 });" << std::endl;
        mHtml << "                i.children('.ba-caption_br').css({ top:    height * 0.5, left:  width * 0.5 });" << std::endl;
        mHtml << "                // Set border" << std::endl;
        mHtml << "                bwidth = parseInt(i.children('.ba-border_tl').css(\"borderRightWidth\"), 10);" << std::endl;
        mHtml << "                i.children('.ba-border_tl').width (width  * 0.5 - 1 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_tl').height(height * 0.5 - 1 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_tr').width (width  * 0.5 - 4 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_tr').height(height * 0.5 - 1 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_bl').width (width  * 0.5 - 1 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_bl').height(height * 0.5 - 4 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_br').width (width  * 0.5 - 4 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_br').height(height * 0.5 - 4 - 2*(bwidth-1));" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-border_tl').css({ top: 0,                left: 0              });" << std::endl;
        mHtml << "                i.children('.ba-border_tr').css({ top: 0,                left: width * 0.5 + 2});" << std::endl;
        mHtml << "                i.children('.ba-border_bl').css({ top: height * 0.5 + 2, left: 0              });" << std::endl;
        mHtml << "                i.children('.ba-border_br').css({ top: height * 0.5 + 2, left: width * 0.5 + 2});" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "            }).mousemove(function (e) {" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                var o = options;" << std::endl;
        mHtml << "                var i = $(this);" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                right_border_width = parseInt(i.children('.ba-layer_tl').css(\"borderRightWidth\"), 10);" << std::endl;
        mHtml << "                bottom_border_width = parseInt(i.children('.ba-layer_tl').css(\"borderBottomWidth\"), 10);" << std::endl;
        mHtml << "                pos_imgX = i.position()['left'];" << std::endl;
        mHtml << "                pos_imgY = i.position()['top'];" << std::endl;
        mHtml << "                pos_mouseX = e.pageX - right_border_width * 0.5;" << std::endl;
        mHtml << "                pos_mouseY = e.pageY - bottom_border_width * 0.5;" << std::endl;
        mHtml << "                new_lwidth  = pos_mouseX - pos_imgX; // left width" << std::endl;
        mHtml << "                new_theight = pos_mouseY - pos_imgY; // top height" << std::endl;
        mHtml << "                img_width   = i.width();" << std::endl;
        mHtml << "                img_height  = i.height();" << std::endl;
        mHtml << "                new_rwidth  = img_width  - new_lwidth;  // right width" << std::endl;
        mHtml << "                new_bheight = img_height - new_theight; // bottom height" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                img_cap_tl = i.children('img:eq(0)').attr('alt');" << std::endl;
        mHtml << "                img_cap_tr = i.children('img:eq(1)').attr('alt');" << std::endl;
        mHtml << "                img_cap_bl = i.children('img:eq(2)').attr('alt');" << std::endl;
        mHtml << "                img_cap_br = i.children('img:eq(3)').attr('alt');" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-layer_tl').width (new_lwidth );" << std::endl;
        mHtml << "                i.children('.ba-layer_tl').height(new_theight);" << std::endl;
        mHtml << "                i.children('.ba-layer_tr').height(new_theight);" << std::endl;
        mHtml << "                i.children('.ba-layer_bl').width (new_lwidth );" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-caption_tl').css({ bottom: new_bheight, right: new_rwidth });" << std::endl;
        mHtml << "                i.children('.ba-caption_tr').css({ bottom: new_bheight, left:  new_lwidth });" << std::endl;
        mHtml << "                i.children('.ba-caption_bl').css({ top:    new_theight, right: new_rwidth });" << std::endl;
        mHtml << "                i.children('.ba-caption_br').css({ top:    new_theight, left:  new_lwidth });" << std::endl;
        mHtml << "                // Set border" << std::endl;
        mHtml << "                bwidth = parseInt(i.children('.ba-border_tl').css(\"borderRightWidth\"), 10);" << std::endl;
        mHtml << "                i.children('.ba-border_tl').width (new_lwidth  - 1 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_tl').height(new_theight - 1 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_tr').width (new_rwidth  - 4 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_tr').height(new_theight - 1 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_bl').width (new_lwidth  - 1 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_bl').height(new_bheight - 4 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_br').width (new_rwidth  - 4 - 2*(bwidth-1));" << std::endl;
        mHtml << "                i.children('.ba-border_br').height(new_bheight - 4 - 2*(bwidth-1));" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-border_tl').css({ top: 0,               left: 0             });" << std::endl;
        mHtml << "                i.children('.ba-border_tr').css({ top: 0,               left: new_lwidth + 2});" << std::endl;
        mHtml << "                i.children('.ba-border_bl').css({ top: new_theight + 2, left: 0             });" << std::endl;
        mHtml << "                i.children('.ba-border_br').css({ top: new_theight + 2, left: new_lwidth + 2});" << std::endl;
        mHtml << "            });" << std::endl;
        mHtml << "        }" << std::endl;
        mHtml << "    });" << std::endl;
        mHtml << "    })(jQuery);" << std::endl;
        mHtml << "-->" << std::endl;
        mHtml << "</script>" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "<script type=\"text/javascript\">" << std::endl;
        mHtml << "$(function () {" << std::endl;
        mHtml << "  $('.cross_compare').qbeforeafter({defaultgap:50, leftgap:0, rightgap:10, caption: true, reveal: 0.5});" << std::endl;
        mHtml << "});" << std::endl;
        mHtml << "</script>" << std::endl;
        mHtml << "<style type=\"text/css\">" << std::endl;
        mHtml << ".ba-layer_tl {position:absolute; top:0; left:0; z-index:3; border-right:3px solid #333; border-bottom:3px solid #333;}" << std::endl;
        mHtml << ".ba-layer_tr {position:absolute; top:0; left:0; z-index:2; border-bottom:3px solid #333;}" << std::endl;
        mHtml << ".ba-layer_bl {position:absolute; top:0; left:0; z-index:1; border-right:3px solid #333;}" << std::endl;
        mHtml << ".ba-layer_br {position:absolute; top:0; left:0; z-index:0;}" << std::endl;
        mHtml << "" << std::endl;
        mHtml << ".ba-border_tl {position:absolute; top:0; left:0; z-index:4;}" << std::endl;
        mHtml << ".ba-border_tr {position:absolute; top:0; left:0; z-index:4;}" << std::endl;
        mHtml << ".ba-border_bl {position:absolute; top:0; left:0; z-index:4;}" << std::endl;
        mHtml << ".ba-border_br {position:absolute; top:0; left:0; z-index:4;}" << std::endl;
        mHtml << "" << std::endl;
        mHtml << ".ba-caption_tl {position:absolute; bottom:10px; right:10px; z-index:120; color:#fff; text-align:center; padding:5px; font-size:12px; font-family:arial; display:none;}" << std::endl;
        mHtml << ".ba-caption_tr {position:absolute; bottom:10px; left: 10px; z-index:120; color:#fff; text-align:center; padding:5px; font-size:12px; font-family:arial; display:none;}" << std::endl;
        mHtml << ".ba-caption_bl {position:absolute; top:10px;    right:10px; z-index:120; color:#fff; text-align:center; padding:5px; font-size:12px; font-family:arial; display:none;}" << std::endl;
        mHtml << ".ba-caption_br {position:absolute; top:10px;    left: 10px; z-index:120; color:#fff; text-align:center; padding:5px; font-size:12px; font-family:arial; display:none;}" << std::endl;
        mHtml << "</style>" << std::endl;
        mHtml << "</head>" << std::endl;
        mHtml << "<body>" << std::endl;
    }

    void AddScene(const std::string &aSceneName)
    {
        mHtml << "<table";

        if(mAlgorithmCount < 100)
            mHtml << " width=\"" << mAlgorithmCount * (mThumbnailSize + 10) << "px\"";

        mHtml << "><tr><td colspan=\"" << mAlgorithmCount << "\"><h2>" << aSceneName << "</h2></td></tr>" << std::endl;
        mHtml << "<tr>" << std::endl;
    }

    void AddRendering(
        const std::string &aMethodName,
        const std::string &aFileName,
        float             aTime,
        BorderColor       aBorderColor = kNone,
        const std::string &aOtherInfo = "")
    {
        // The image
        mHtml << "<td valign=\"top\" align=\"center\">"
            << "<div style=\""
            << "width:"<< mThumbnailSize + 10 <<"px;line-height:90%;\">"
            << " <a href=\"" << aFileName << "\">";
#if 1
        mHtml << "<img src=\"" << aFileName << "\" "
            << "width=\""<< mThumbnailSize <<"px\"  ";
        if(aBorderColor == kGreen)
            mHtml << "style=\"border:5px solid #0c0\" ";
        else if(aBorderColor == kRed)
            mHtml << "style=\"border:5px solid #f00\" ";
        else
            mHtml << "style=\"border:5px solid #ccc\" ";
        mHtml << " alt=\"" << aFileName << " (" << MakeMessage("%.2f", aTime) << " s)\" ";
        mHtml << "height=\""<< mThumbnailSize <<"px\" />";
#else
        mHtml << "<div style=\"background: url(" << aFileName
            << "); background-size: 128px;";
        if(aBorderColor == kRed)
        {
            mHtml << "background-position: -5px -5px; border: 5px solid #f00; "
                << "width:"<< mThumbnailSize - 10 <<"px; "
                << "height:"<< mThumbnailSize - 10 <<"px;\"></div>";
        }
        else if(aBorderColor == kGreen)
        {
            mHtml << "background-position: -5px -5px; border: 5px solid #0c0; "
                << "width:"<< mThumbnailSize - 10 <<"px; "
                << "height:"<< mThumbnailSize - 10 <<"px;\"></div>";
        }
        else // None
        {
            mHtml << "width:"<< mThumbnailSize <<"px; "
                << "height:"<< mThumbnailSize <<"px;\"></div>";
        }
#endif
        mHtml << "</a>" << std::endl;

        // The text
        mHtml << "<br/><small>" << aMethodName
            << " (" << MakeMessage("%.2f", aTime) << " s)" << aOtherInfo
            << "</small></div></td>" << std::endl;
    }

    void AddFourWaySplit(
        const std::string aMethodFiles[4],
        const std::string aMethodNames[4],
        const int         aBorderColors[4],
        const int         aSize)
    {
        mHtml << "</tr><tr>" << std::endl;
        mHtml << "<td colspan=\"" << mAlgorithmCount << "\" align=\"center\">" << std::endl;
        mHtml << "<div class=\"cross_compare\" style=\"width:" << aSize
            << "px;height:" << aSize << "px;cursor:crosshair\">" << std::endl;
        for(int i=0; i<4; i++)
        {
            mHtml << "<img src=\"" << aMethodFiles[i]
            << "\" alt=\"" << aMethodNames[i]
            << "\" width=\"" << aSize
            << "\" height=\"" << aSize << "\" ";
            if(aBorderColors[i] == kGreen)
                mHtml << "style=\"border:2px solid #0c0\"/>" << std::endl;
            else if(aBorderColors[i] == kRed)
                mHtml << "style=\"border:2px solid #f00\"/>" << std::endl;
            else
                mHtml << "style=\"border:2px solid #ccc\"/>" << std::endl;
        }
        mHtml << "</div>" << std::endl;
        mHtml << "</td>" << std::endl;
        mHtml << "</tr></table>" << std::endl;
    }

    // Taken from: http://www.tin.org/bin/man.cgi?section=3&topic=vsnprintf
    std::string MakeMessage(const char *fmt, ...)
    {
        /* Guess we need no more than 100 bytes. */
        int         size = 100;
        std::string str;
        va_list     ap;

        while (1) {
            str.resize(size);
            /* Try to print in the allocated space. */
            va_start(ap, fmt);
            int n = vsnprintf((char*)str.c_str(), size, fmt, ap);
            va_end(ap);
            /* If that worked, return the string. */
            if (n > -1 && n < size)
            {
                str.resize(n);
                return str;
            }
            /* Else try again with more space. */
            if (n > -1)     /* glibc 2.1 */
                size = n+1; /* precisely what is needed */
            else            /* glibc 2.0 */
                size *= 2;  /* twice the old size */
        }
    }

public:

    int           mAlgorithmCount;
    int           mThumbnailSize;
    std::string   mFileName;
    std::ofstream mHtml;
};

#pragma warning(pop)

#endif //__HTML_WRITER_HXX__
