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

#include <vector>
#include <cmath>
#include <fstream>
#include <string>

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
    HtmlWriter(const std::string& aFileName) : mFileName(aFileName), mHtml(mFileName)
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

    void WriteHeader()
    {
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
        mHtml << "                new_width  = pos_mouseX - pos_imgX;" << std::endl;
        mHtml << "                new_height = pos_mouseY - pos_imgY;" << std::endl;
        mHtml << "                img_width  = i.width();" << std::endl;
        mHtml << "                img_height = i.height();" << std::endl;
        mHtml << "                img_cap_tl = i.children('img:eq(0)').attr('alt');" << std::endl;
        mHtml << "                img_cap_tr = i.children('img:eq(1)').attr('alt');" << std::endl;
        mHtml << "                img_cap_bl = i.children('img:eq(2)').attr('alt');" << std::endl;
        mHtml << "                img_cap_br = i.children('img:eq(3)').attr('alt');" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-layer_tl').width(new_width);" << std::endl;
        mHtml << "                i.children('.ba-layer_tl').height(new_height);" << std::endl;
        mHtml << "                i.children('.ba-layer_tr').height(new_height);" << std::endl;
        mHtml << "                i.children('.ba-layer_bl').width(new_width);" << std::endl;
        mHtml << "" << std::endl;
        mHtml << "                i.children('.ba-caption_tl').css({ bottom: img_height - new_height, right: img_width - new_width });" << std::endl;
        mHtml << "                i.children('.ba-caption_tr').css({ bottom: img_height - new_height, left:  new_width });" << std::endl;
        mHtml << "                i.children('.ba-caption_bl').css({ top:    new_height,              right: img_width - new_width });" << std::endl;
        mHtml << "                i.children('.ba-caption_br').css({ top:    new_height,              left:  new_width });" << std::endl;
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
        float aTime,
        BorderColor aBorderColor = kNone,
        const std::string &aOtherInfo = "")
    {
        // The image
        mHtml << "<td valign=\"top\" align=\"center\">"
            << " <a href=\"" << aFileName << "\">";
        mHtml << "<img src=\"" << aFileName << "\" "
            << "width=\""<< mThumbnailSize <<"px\"  ";
#if 1
        if(aBorderColor == kGreen)
            mHtml << "style=\"border:5px solid #0c0\" ";
        else if(aBorderColor == kRed)
            mHtml << "style=\"border:5px solid #f00\" ";
        else
            mHtml << "style=\"border:5px solid #ccc\" ";
        mHtml << " alt=\"" << aFileName << " (" << aTime << " s)\" ";
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
            << " (" << aTime << " s)" << aOtherInfo
            << "</small></td>" << std::endl;
    }

    void AddFourWaySplit(
        const std::string aMethodFiles[4],
        const std::string aMethodNames[4],
        const int aSize)
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
            << "\" height=\"" << aSize << "\"/>" << std::endl;
        }
        mHtml << "</div>" << std::endl;
        mHtml << "</td>" << std::endl;
        mHtml << "</tr></table>" << std::endl;
    }

public:
    int           mAlgorithmCount;
    int           mThumbnailSize;
    std::string   mFileName;
    std::ofstream mHtml;
};

#endif //__HTML_WRITER_HXX__
